import spacy

class GrammarDecorator:
    def __init__(self):
        # Load the small, efficient English model
        # disable components we don't need for speed (NER, etc.)
        self.nlp = spacy.load("en_core_web_sm", disable=["ner", "textcat"])
        
        # Define our tags
        self.TAGS = {
            "PAST": "[PAST]",
            "PROG": "[PROG]",  # Progressive (is running)
            "PERF": "[PERF]",  # Perfect (has run)
            "PASS": "[PASS]",  # Passive (was eaten)
            "NEG":  "[NEG]"    # Negation (did not)
        }

    def process(self, text):
        doc = self.nlp(text)
        output_tokens = []
        
        # We need to identify which tokens to SKIP (auxiliaries that get converted to tags)
        # We store the indices of tokens we have "consumed" as tags.
        consumed_indices = set()

        # First pass: Identify auxiliaries and build tag maps for their head verbs
        verb_tags = {} # Key: verb_token_index, Value: set of tags strings

        for token in doc:
            # 1. Handle Negation ("not")
            if token.dep_ == "neg":
                head_idx = token.head.i
                if head_idx not in verb_tags: verb_tags[head_idx] = set()
                verb_tags[head_idx].add(self.TAGS["NEG"])
                consumed_indices.add(token.i)
            
            # 2. Handle Auxiliaries (is, has, was, did)
            elif token.dep_ in ["aux", "auxpass"]:
                head_verb = token.head
                
                # Passive Voice check ("was eaten")
                if token.dep_ == "auxpass":
                    if head_verb.i not in verb_tags: verb_tags[head_verb.i] = set()
                    verb_tags[head_verb.i].add(self.TAGS["PASS"])
                    
                    # If the auxiliary itself is past tense ("was"), mark PAST
                    if token.tag_ in ["VBD", "VBN"]:
                        verb_tags[head_verb.i].add(self.TAGS["PAST"])
                    consumed_indices.add(token.i)

                # Active Auxiliaries
                elif token.dep_ == "aux":
                    if head_verb.i not in verb_tags: verb_tags[head_verb.i] = set()
                    
                    # Perfect Aspect check ("has run")
                    if token.lemma_ == "have":
                        verb_tags[head_verb.i].add(self.TAGS["PERF"])
                        consumed_indices.add(token.i)
                    
                    # Progressive Aspect check ("is running")
                    elif token.lemma_ == "be" and head_verb.tag_ == "VBG":
                        verb_tags[head_verb.i].add(self.TAGS["PROG"])
                        # If aux is past ("was running"), add PAST
                        if token.tag_ == "VBD":
                            verb_tags[head_verb.i].add(self.TAGS["PAST"])
                        consumed_indices.add(token.i)
                        
                    # "Do" support ("did run")
                    elif token.lemma_ == "do":
                        # If "did", mark past
                        if token.tag_ == "VBD":
                            verb_tags[head_verb.i].add(self.TAGS["PAST"])
                        consumed_indices.add(token.i)
                    
                    # Future "will" - usually keep "will" as a modal, 
                    # or you can map it to a [FUT] tag if you prefer.
                    # Here we let "will" pass through as a token if it's not consumed.

        # Second pass: Construct the final sequence
        for token in doc:
            if token.i in consumed_indices:
                continue

            # If this is a VERB that has accumulated tags
            if token.pos_ == "VERB":
                # Always use lemma for the verb
                output_tokens.append(token.lemma_)
                
                # Check for intrinsic past tense (e.g., "ran" without aux)
                if token.tag_ == "VBD":
                    if token.i not in verb_tags: verb_tags[token.i] = set()
                    verb_tags[token.i].add(self.TAGS["PAST"])

                # Append any tags associated with this verb
                if token.i in verb_tags:
                    # Sort tags to ensure deterministic order (important for LLM consistency)
                    sorted_tags = sorted(list(verb_tags[token.i]))
                    output_tokens.extend(sorted_tags)
            
            else:
                # Non-verbs: Keep original text or lemma? 
                # Usually text is better for nouns (proper nouns), 
                # but lemma is better for plurals -> singulars.
                # Let's use text for simplicity, or lemma for compression.
                # Here we use text to preserve names properly.
                output_tokens.append(token.text)

        return output_tokens

# --- TEST SUITE ---
decorator = GrammarDecorator()

test_sentences = [
    "John runs fast.",
    "John ran fast.",
    "John is running fast.",
    "John was running fast.",
    "John has run fast.",
    "John did not run.",
    "The apple was eaten by John."
]

print(f"{'ORIGINAL':<30} | {'DECORATED OUTPUT'}")
print("-" * 70)
for sent in test_sentences:
    result = decorator.process(sent)
    print(f"{sent:<30} | {result}")