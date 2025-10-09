# Data structures (minimal, no magic)
@dataclass
class AgendaItem:
    id: str
    description: str
    type: str
    created_at: int
    last_executed: int
    deadline: Optional[int]
    base_priority: float
    source_type: str
    source_id: Optional[str]
    status: str

class Agenda:
    items: List[AgendaItem]

# Extraction functions (LLM-based, clearly defined)
def extract_agenda_items(discourse_state: str, my_name: str) -> List[dict]:
    """
    Extract all items competing for my attention.
    """
    
    prompt = f"""
Extract agenda items from this discourse state for {my_name}.

DISCOURSE STATE:
{discourse_state}

Create agenda items for:
1. [Self → Other] commitments - things {my_name} committed to DO
2. [Other → Self] commitments - things others committed to do that {my_name} should MONITOR
3. Mutual agreements - joint ACTIVITIES both parties will pursue

Skip vague cooperation statements. Focus on concrete, actionable items.

Output format (one per line):
TYPE|ACTION_DESCRIPTION

Where TYPE is: commitment, monitoring, or activity

Examples:
commitment|Give Joe water from the spring
activity|Explore surroundings with Joe to find way out  
monitoring|Check that Joe is participating in exploration

Output:"""
    
    response = llm_call(prompt)
    
    items = []
    for line in response.strip().split('\n'):
        if '|' not in line:
            continue
        type_, action = line.split('|', 1)
        items.append({
            "type": type_.strip(),
            "action": action.strip()
        })
    
    return itemsdef discourse_to_agenda_items(discourse_state, agenda) -> List[AgendaItem]:
    # One LLM prompt, string parsing
    pass

def activity_to_agenda_item(activity) -> AgendaItem:
    # Direct conversion from structured activity
    pass

# Update functions  
def check_discourse_completions(discourse_state, agenda):
    # One LLM prompt to find completed items
    pass

def after_action_executed(action_result, agenda):
    # Mark executed, check if complete
    pass

# Integration
def reflection_cycle(turns, agenda, discourse_state):
    # Ties everything together
    pass