# Desired Tool Capabilities for LLM Agents — Working Checklist

This is my running list of tool-use capabilities I want an LLM agent
to have, organized by category. I'm maintaining this as a reference
for evaluating both our own system and the literature.

## Information retrieval
- Web search with query formulation and result filtering
- Academic paper search (Semantic Scholar, arXiv, Google Scholar)
- PDF fetching and structured extraction
- Web page fetching with content extraction (not just raw HTML)
- API querying with authentication and pagination

## Knowledge management
- Create, read, update, delete persistent notes
- Organize notes into named collections
- Semantic search over existing notes
- Cross-reference and link related notes
- Summarization and synthesis across multiple notes

## Code execution
- Write and execute Python in a sandboxed environment
- Install packages on demand
- Read and write files on the local filesystem
- Parse structured data (CSV, JSON, XML)
- Generate visualizations (plots, charts, diagrams)

## Reasoning and analysis
- Multi-step planning with dependency tracking
- Claim verification against multiple sources
- Comparative analysis (structured pros/cons, feature matrices)
- Quantitative reasoning with calculation
- Argument mapping (supporting/contradicting evidence)

## Communication
- Produce structured output in specified formats
- Adjust detail level based on audience
- Cite sources with verifiable references
- Flag uncertainty and confidence levels explicitly

## Self-management
- Track progress on multi-step tasks
- Detect and recover from errors
- Know when to ask for clarification vs. proceed
- Manage resource budgets (API calls, compute time)
- Learn from past execution failures

## Integration
- Calendar and scheduling
- Email composition and sending
- Version control operations (git)
- Database querying
- External service webhooks and notifications

## Gaps I'm particularly interested in
- Real-time monitoring and alerting (not just one-shot tasks)
- Collaborative multi-agent coordination
- Long-horizon planning with replanning on failure
- Proactive information gathering without explicit user request
- Tool creation — building new tools from existing primitives
