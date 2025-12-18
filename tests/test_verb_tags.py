import spacy

class GrammarDecorator:
    def __init__(self):
        # Load the small, efficient English model
        # disable components we don't need for speed
        self.nlp = spacy.load("en_core_web_sm", disable=["ner", "textcat"])
        
        # Define our tags
        self.TAGS = {
            "PAST": "[PAST]",
            "PROG": "[PROG]",  # Progressive (is running)
            "PERF": "[PERF]",  # Perfect (has run)
            "PASS": "[PASS]",  # Passive (was eaten)
            "NEG":  "[NEG]"    # Negation (did not)
        }
        
        # The "Ghost" verbs we want to delete and replace with tags entirely
        self.STRUCTURAL_VERBS = {"be", "have", "do"}

    def process(self, text):
        doc = self.nlp(text)
        output_tokens = []
        
        # We need to identify which tokens to SKIP (auxiliaries that get converted to tags)
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
            
            # 2. Handle Auxiliaries (is, has, was, did acting as helpers)
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
                        if token.tag_ == "VBD": # was running
                            verb_tags[head_verb.i].add(self.TAGS["PAST"])
                        consumed_indices.add(token.i)
                        
                    # "Do" support ("did run")
                    elif token.lemma_ == "do":
                        if token.tag_ == "VBD": # did run
                            verb_tags[head_verb.i].add(self.TAGS["PAST"])
                        consumed_indices.add(token.i)

        # Second pass: Construct the final sequence
        for token in doc:
            if token.i in consumed_indices:
                continue

            # If this is a VERB
            if token.pos_ == "VERB" or token.pos_ == "AUX":
                
                # SPECIAL CASE: Is this a "Main Verb" that is also a Structural Verb?
                # e.g. "John is 80" -> lemma "be"
                if token.lemma_ in self.STRUCTURAL_VERBS:
                    # Capture tense info before we delete the word
                    tags_to_add = set()
                    if token.i in verb_tags:
                        tags_to_add.update(verb_tags[token.i])
                    
                    if token.tag_ in ["VBD", "VBN"]:
                        tags_to_add.add(self.TAGS["PAST"])
                    
                    # Instead of appending token.lemma_, we append "be" ONLY if it's acting as a Copula (linker)
                    # But for your memory agent, you likely want "be" to exist as a node.
                    # If you want "John is 80" -> "John be 80", we KEEP it.
                    # If you want "John is 80" -> "John 80", we DELETE it.
                    
                    # Let's go with the SAFE route: Normalize everything to "be" / "have" / "do"
                    output_tokens.append(token.lemma_)
                    
                    if tags_to_add:
                        output_tokens.extend(sorted(list(tags_to_add)))

                else:
                    # Regular Content Verb (run, eat, fly)
                    output_tokens.append(token.lemma_)
                    
                    # Check for intrinsic past tense (e.g., "ran")
                    if token.tag_ == "VBD":
                        if token.i not in verb_tags: verb_tags[token.i] = set()
                        verb_tags[token.i].add(self.TAGS["PAST"])

                    # Append tags
                    if token.i in verb_tags:
                        sorted_tags = sorted(list(verb_tags[token.i]))
                        output_tokens.extend(sorted_tags)
            
            else:
                # Non-verbs
                output_tokens.append(token.text)

        return output_tokens

# --- TEST SUITE ---
decorator = GrammarDecorator()

test_sentences = [
    "John is 80 years old.",       # The Copula Case
    "John was 80 years old.",      # The Past Copula Case
    "John has a car.",             # Main verb 'have'
    "John had a car.",             # Past main verb 'have'
    "John does his homework.",     # Main verb 'do'
    "John runs fast."              # Regular verb
]

print(f"{'ORIGINAL':<30} | {'DECORATED OUTPUT'}")
print("-" * 70)
for sent in test_sentences:
    result = decorator.process(sent)
    print(f"{sent:<30} | {result}")