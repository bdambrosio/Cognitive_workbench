"""
Plan Guidance - Retrieve similar past plans based on goal similarity.

Uses embedding-based search to find historically successful plans for similar goals.
"""
import json
import logging
import os
import random
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import numpy as np

logger = logging.getLogger(__name__)


class PlanGuidance:
    """
    Retrieves similar past plans based on goal text similarity.
    
    Loads feedback history, builds embeddings for goals, and searches for
    similar plans using cosine similarity. Results are scored by outcome,
    error count, and plan length.
    """
    
    def __init__(self, resource_manager, feedback_file_path: Optional[str] = None):
        """
        Initialize plan guidance system.
        
        Args:
            resource_manager: InfospaceResourceManager instance (for embedder)
            feedback_file_path: Optional path to feedback JSONL file
                               (defaults to src/data/planner_feedback/plan_guidance.jsonl)
        """
        self.resource_manager = resource_manager
        self.feedback_records: List[Dict] = []
        self.goal_embeddings: List[np.ndarray] = []
        self._index_built = False
        
        # Determine feedback file path
        if feedback_file_path is None:
            base_dir = os.path.dirname(__file__)
            feedback_file_path = os.path.join(base_dir, 'data', 'planner_feedback', 'plan_guidance.jsonl')
        
        self.feedback_file_path = Path(feedback_file_path)
        
        # Load feedback history
        self._load_feedback_history()
    
    def _load_feedback_history(self):
        """Load feedback records from JSONL file."""
        if not self.feedback_file_path.exists():
            logger.warning(f"Feedback file not found: {self.feedback_file_path}")
            return
        
        self.feedback_records = []
        plan_lengths = []
        action_type_counts = {}
        
        try:
            with open(self.feedback_file_path, 'r', encoding='utf-8') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        record = json.loads(line)
                        # Validate required fields
                        if 'goal' in record and 'outcome' in record:
                            self.feedback_records.append(record)
                            
                            # Count plan steps and action types
                            plan = record.get('plan', [])
                            if isinstance(plan, list):
                                plan_length = len(plan)
                                plan_lengths.append(plan_length)
                                
                                # Count action types in this plan
                                for action in plan:
                                    if isinstance(action, dict):
                                        action_type = action.get('type')
                                        if action_type:
                                            action_type_counts[action_type] = action_type_counts.get(action_type, 0) + 1
                        else:
                            logger.warning(f"Skipping invalid record at line {line_num}: missing goal or outcome")
                    except json.JSONDecodeError as e:
                        logger.warning(f"Skipping malformed JSON at line {line_num}: {e}")
            
            # Log statistics
            num_plans = len(self.feedback_records)
            if num_plans > 0:
                avg_plan_length = sum(plan_lengths) / len(plan_lengths) if plan_lengths else 0.0
                logger.info(f"Loaded {num_plans} feedback records from {self.feedback_file_path}")
                logger.info(f"Statistics: {num_plans} plans, avg plan length: {avg_plan_length:.1f} steps")
                
                if action_type_counts:
                    logger.info("Action type occurrences:")
                    # Sort by count (descending) for readability
                    sorted_types = sorted(action_type_counts.items(), key=lambda x: x[1], reverse=True)
                    for action_type, count in sorted_types:
                        logger.info(f"  {action_type}: {count}")
            else:
                logger.info(f"Loaded {num_plans} feedback records from {self.feedback_file_path}")
        except Exception as e:
            logger.error(f"Error loading feedback history: {e}")
            self.feedback_records = []
    
    def _build_index(self):
        """Build embedding index for all goals (lazy initialization)."""
        if self._index_built:
            return
        
        if not self.feedback_records:
            logger.debug("No feedback records to index")
            self._index_built = True
            return
        
        self.goal_embeddings = []
        valid_records = []
        
        try:
            for record in self.feedback_records:
                goal = record.get('goal', '')
                if not goal:
                    continue
                
                # Generate embedding using resource_manager's embedder
                embedding = self.resource_manager._generate_embedding(goal)
                embedding_array = np.array(embedding, dtype='float32')
                
                # Normalize for cosine similarity
                norm = np.linalg.norm(embedding_array)
                if norm > 0:
                    embedding_array = embedding_array / norm
                
                self.goal_embeddings.append(embedding_array)
                valid_records.append(record)
            
            # Update records to only include indexed ones
            self.feedback_records = valid_records
            self._index_built = True
            logger.info(f"Built embedding index for {len(self.goal_embeddings)} goals")
        except Exception as e:
            logger.error(f"Error building embedding index: {e}")
            self.goal_embeddings = []
            self._index_built = True
    
    def _score_plan(self, record: Dict, similarity: float) -> float:
        """
        Score a plan record for ranking.
        
        Scoring formula:
        - Base: 1000.0 if outcome=True, 0.0 if outcome=False
        - Penalties: error_count * 10.0, error_rate * 5.0
        
        Args:
            record: Feedback record dict
            
        Returns:
            Score (higher is better)
        """
        outcome = record.get('outcome', False)
        plan = record.get('plan', [])
        error_count = record.get('error_count', 0)
        
        plan_length = len(plan) if isinstance(plan, list) else 0
        
        # Base score: outcome dominates
        base_score = 1000.0 if outcome else 0.0
        
        # Penalties
        plan_length_penalty = plan_length * 2.0
        error_penalty = error_count * 50.0
        
        # Normalized error rate penalty
        if plan_length > 0:
            error_rate = error_count / plan_length
            error_rate_penalty = error_rate * 100.0
        else:
            error_rate_penalty = 0.0

        score_bonus = 25.0*similarity
        
        score = base_score - plan_length_penalty - error_penalty - error_rate_penalty + score_bonus
        return score
    
    def find_similar_plans(self, goal: str, limit: int = 1, threshold: float = 0.3) -> List[Dict]:
        """
        Find similar past plans based on goal text.
        
        Args:
            goal: Goal text to search for
            limit: Maximum number of results to return (default: 1)
            threshold: Minimum cosine similarity threshold (default: 0.3)
            
        Returns:
            List of feedback records, sorted by score (best first)
        """
        # Build index if needed
        self._build_index()
        
        if not self.goal_embeddings or not self.feedback_records:
            logger.debug("No indexed plans available for search")
            return []
        
        try:
            # Generate embedding for query goal
            query_embedding = self.resource_manager._generate_embedding(goal)
            query_array = np.array(query_embedding, dtype='float32')
            
            # Normalize for cosine similarity
            norm = np.linalg.norm(query_array)
            if norm == 0:
                logger.warning("Query embedding has zero norm")
                return []
            query_array = query_array / norm
            
            # Compute similarities
            similarities = []
            for i, goal_embedding in enumerate(self.goal_embeddings):
                similarity = float(np.dot(query_array, goal_embedding))
                if similarity >= threshold and self.feedback_records[i]['goal'] != goal and self.feedback_records[i]['error_count'] == 0:
                    record = self.feedback_records[i].copy()
                    record['_similarity'] = similarity
                    record['_score'] = self._score_plan(record, similarity)
                    similarities.append(record)
            
            # Sort by score (descending), then by similarity (descending)
            similarities.sort(key=lambda x: (x['_score'], x['_similarity']), reverse=True)
            
            # If there are more plans tied for top score than limit, shuffle them randomly
            if len(similarities) > limit:
                top_score = similarities[0]['_score']
                # Find all plans with top score
                tied_plans = [s for s in similarities if s['_score'] == top_score]
                if len(tied_plans) > limit:
                    # Shuffle tied plans randomly
                    random.shuffle(tied_plans)
                    # Replace top plans with shuffled tied plans
                    similarities[:len(tied_plans)] = tied_plans
            
            # Return top N
            results = similarities[:limit]
            logger.debug(f"Found {len(results)} similar plans (threshold={threshold}, limit={limit})")
            return results
            
        except Exception as e:
            logger.error(f"Error searching for similar plans: {e}")
            return []

