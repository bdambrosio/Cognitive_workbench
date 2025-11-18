import requests

paperId = "649def34f8be52c8b66281af98ae884c09aef38b"

# Define the API endpoint URL
url = f"http://api.semanticscholar.org/graph/v1/paper/search?query=constitutional+AI&limit=5&fields=paperId,title"

# Define the query parameters
query_params = {"fields": "title,year,abstract,citationCount"}

# Directly define the API key (Reminder: Securely handle API keys in production environments)
api_key = "your api key goes here"  # Replace with the actual API key

# Define headers with API key
headers = {}

# Send the API request
response = requests.get(url, params=query_params, headers=headers)

# Check response status
if response.status_code == 200:
   response_data = response.json()
   # Process and print the response data as needed
   print(response_data)
else:
   print(f"Request failed with status code {response.status_code}: {response.text}")
   
