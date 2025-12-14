from datasets import load_dataset

# Load dataset
ds = load_dataset("lmsys/lmsys-chat-1m", split="train")

# Extract only the first user prompt from each conversation
user_inputs = [
    conv[0]['content'] 
    for conv in ds['conversation'] 
    if conv[0]['role'] == 'user'
]

print(user_inputs[:5])
