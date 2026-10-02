# Rated by the buyer: material
<!-- The buyer, at intake (illustrative, written by the practice for the demo):
     - How easy is it to integrate into our TypeScript app?
     - Can each of our customers have their own links, kept separate from the others?
     - Can we run more than one copy behind a load balancer?
     - Will it stand up to someone abusing link creation? -->
The API is described in a machine-readable specification, such as an OpenAPI document, kept in the repository.
The server sends CORS headers that let a web application on another origin call the API.
Links can be kept per customer, with each customer's access limited to its own links.
The service can run as more than one instance sharing the same stored links.
Requests that create links are rate-limited.
