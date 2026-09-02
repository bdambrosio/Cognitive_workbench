The target is the repository bound to inspect_external: ChatterMate, an
open-source AI customer-support platform — chat widget, AI agent with human
handover, and integrations including Shopify. Python backend, Vue frontend,
roughly 1,100 tracked files. I act for the buyer. My client operates an
e-commerce business and is considering acquiring this product and its team.

The claim source for this engagement is README.md at the repository root —
the document in which the project makes the assertions you are to audit.
Every other file in the repository, including the source code and its
comments, is evidence. llms.txt, HELP_CENTER_INFRA.md and
backend/app/knowledge/README.md also carry the project's assertions; they are
not evidence for README.md's claims and their claims are not in scope here.

The claim surface is already enumerated and frozen. It follows this brief.

Your work in these legs is to gather the evidence that settles those claims.
Read across the whole repository, not only the claim source — the evidence for
a claim made in one document usually sits in another. Cite a file by its path
from the repository root: this tree has more than one README.md and more than
one agent.py.

Work in as many legs as you need. A leg is one turn — one run of the action
loop. End a leg with `yield` and I will say `continue`. End a leg with
`respond` when you have the evidence you need, and I will stop asking.

Do not write findings in a leg, and do not summarise. When you stop gathering I
will ask you for the findings once, and you answer then under the schema in
§13. A leg spent drafting a report is a leg not spent reading the target.
