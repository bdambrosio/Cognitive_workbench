# Knowledge Base Curation System

## What this is

An architectural sketch for extending Jill with persistent, LLM-maintained knowledge bases stored in Obsidian. The core idea: Bruce clips web articles and drops documents into Obsidian; Jill compiles them into structured wikis, maintains those wikis over time, and answers questions against them. Obsidian is the durable store and the viewing frontend. Jill is the compiler, curator, and query engine.

Inspired by Karpathy's description of LLM-compiled personal knowledge bases, adapted to Cognitive Workbench's concern/task/goal architecture.

## Three-tier agent memory

Context, infospace, and Obsidian form three tiers of agent memory, analogous to registers, cache, and disk.

**Context** is what Jill can see right now — the OODA snapshot, current goal, tool results. Small, expensive, volatile.

**Infospace** (Notes and Collections) is working memory that persists across goals within a session. Notes hold individual chunks of content. Collections are ordered references to Notes — structural, not content-bearing. Infospace is Jill's scratchpad: she creates Notes to hold intermediate results and Collections to organize them.

**Obsidian** is durable storage. Files persist indefinitely, are human-readable, and exist independently of Jill's runtime. Obsidian is the authoritative store for knowledge bases. Jill reads from and writes to Obsidian, but Obsidian doesn't depend on Jill being running.

The important architectural consequence: when Jill needs to work with Obsidian content, she must bring it into infospace first. For small notes, that's a single Note. For large compiled wiki pages (potentially hundreds of thousands of words), loading the entire page into a single Note would blow the context budget. This requires structured decomposition at the tool level.

## Large-page decomposition

The Obsidian read tool must handle large pages by decomposing them into Collections, not loading them as monolithic Notes. This is a tool-level behavior, not an agent-level decision — Jill doesn't choose whether to decompose; the tool does it automatically based on content size.

The decomposition works roughly like this:

- **Small pages** (under some threshold — maybe 4K tokens) load as a single Note, same as today.
- **Large pages** load as a Collection. The tool chunks the page by sections (markdown headers are natural boundaries), creates a Note per section, creates an index Note summarizing the page structure with section titles and brief descriptions, and returns a Collection with the index Note first followed by the section Notes. The index Note is the one Jill reads to orient; she pulls section Notes selectively as needed.

This means when Jill searches Obsidian and gets back a large wiki article, she receives a Collection she can navigate rather than a wall of text she can't fit in context. The agent sees structured working memory; the tool handles the mechanics of getting it there.

Open question: should the decomposition be cached? If Jill reads the same large page across multiple goal executions, recreating the Collection each time is wasteful. But caching introduces staleness if the page has been updated. Probably: no caching initially, revisit if performance demands it.

## Obsidian write capability

The existing `search-obsidian` tool uses the Obsidian Local REST API for reads. The same API supports writes (PUT to `/vault/{path}`). Extending the tool to support writes is the main new plumbing.

Write operations Jill needs:

- **Create note at path**: write a new .md file at a specified vault location. This is how Jill creates wiki articles, index files, and output documents.
- **Update note content**: overwrite or append to an existing note. For index maintenance, article revision, backlink updates.
- **List directory**: get the file listing for a vault directory. Jill needs this to understand wiki structure without reading every file. The current search tool may partially cover this, but explicit directory listing is cleaner for structural operations.
- **Delete/move note**: for wiki reorganization during linting. Lower priority — Jill can work around this initially by creating new files and updating references.

Direct write to Obsidian is the right model here. The wiki is an external artifact, not Jill's working memory. Infospace Notes are ephemeral projections used during compilation; the Obsidian vault is the durable output. Trying to mirror the wiki in infospace would create a synchronization problem.

## Sensor: clip-watcher

A code-type sensor watching the Obsidian vault's clippings directory for new `.md` files. When Bruce uses the Obsidian Web Clipper extension, the clipped article lands in a known directory. The sensor detects it.

Disposition: **inform** is probably right. The sensor reports "N new clips since last check: [titles]" on a schedule (5m or so). The operational task picks up unprocessed clips during its own execution cycle. This keeps observation and action cleanly separated — the sensor doesn't trigger compilation directly; it just tells Jill what's new.

If Bruce wants immediate processing, the disposition could be `alert`, which would interrupt the OODA loop and produce a faster response. But for a background curation system, inform-level is the right default.

The sensor needs to track what it's already reported, so it doesn't re-announce old clips. A small state file (last-reported timestamp or set of seen filenames) in the sensor's directory handles this.

## The task: maintain knowledge base

Each research topic Bruce cares about becomes a persistent task: "Maintain knowledge base: [topic]." The task links to a user concern or the knowledge-improvement seed concern.

The task's lifecycle:

- **Establishment**: Jill verifies she can read/write the relevant Obsidian directories, creates the initial wiki structure (index file, directory layout), and processes any existing raw clips.
- **Operational**: The task fires on its cooldown schedule. Each execution cycle is one bounded unit of work.

### Vault layout convention

Each knowledge base lives in a predictable directory structure within the Obsidian vault:

```
vault/
  KnowledgeBases/
    Transformers/
      _index.md          # Jill-maintained: topic overview, article list, structure map
      _processing.md     # Jill-maintained: what's been processed, what's pending
      raw/               # Web clips and source documents land here
        article-1.md
        article-2.md
      wiki/              # Jill-compiled articles
        attention.md
        positional-encoding.md
        ...
      output/            # Q&A outputs, visualizations, filed-back results
```

This structure is specified in the task configuration, not discovered. Jill knows where to find raw clips and where to write compiled articles because the task tells her.

### Execution cycle

Each operational execution follows the normal OODA pattern within a single goal:

1. Read `_index.md` and `_processing.md` to understand current state
2. Check raw/ for unprocessed clips (comparing against processing log)
3. Pick the highest-value action:
   - **Compile**: process a new clip into the wiki — read the clip, determine which concept(s) it covers, create or update wiki articles, update cross-references and backlinks, update the index
   - **Enhance**: update an existing article with better synthesis, fill gaps, improve connections
   - **Lint**: consistency check — find broken links, stale summaries, missing cross-references, contradictions between articles
   - **Index**: rebuild or update the master index and per-article summaries
4. Write results back to Obsidian
5. Update `_processing.md` with what was done

Each cycle does one thing well rather than trying to process everything. Over many cycles, the wiki accumulates quality. This is the milestone loop applied to curation: bounded steps, accumulated state, adaptive next-step selection.

### Compilation detail

When Jill compiles a new clip into the wiki, the sequence within a single goal is roughly:

- Read the raw clip (possibly as a Collection if it's large)
- Read the wiki index to understand existing coverage
- Determine: is this a new concept needing a new article, or additional evidence for existing articles?
- If new article: envision what the article should contain (structure, level of detail, connections to existing articles), then write it
- If existing article update: read the relevant article, integrate the new information, rewrite as needed
- Update backlinks in affected articles
- Update the master index
- Log the processing in `_processing.md`

The envisioning system has a natural role here. Before writing an article, Jill forms a concrete image of what a good article for this concept should look like. The vision evaluator can then assess whether the output matches. This is quality-sensitive generation where envisioning earns its keep.

The similar-plans mechanism also accumulates real value. Every "compile article about [concept]" goal produces a plan with an outcome. Over time, Jill's plan library fills with examples of "how I successfully compiled an article about [type of concept]" — grounded few-shot learning applied to knowledge curation.

## Q&A against the knowledge base

When Bruce asks Jill a question about a topic with a knowledge base, Jill's normal chat-response path works — she searches the wiki via the Obsidian tool, reads relevant articles (decomposed into Collections for large ones), and responds.

The interesting extension from Karpathy's workflow: outputs get filed back. If Bruce asks a complex question and Jill produces a researched answer, that answer could be written to the output/ directory as a standalone document, enriching the knowledge base for future queries. This should probably be a deliberate choice ("file this") rather than automatic — not every Q&A response deserves persistence.

## Linting as a separate sensor

Wiki health checks are a natural fit for a plan-type sensor. Periodically (maybe hourly or daily), a linting sensor:

- Reads the wiki index
- Does a lightweight LLM-based assessment: are summaries consistent with articles? Are there obvious gaps in coverage? Do backlinks resolve? Are there topics mentioned in multiple articles that deserve their own article?
- Reports findings via inform disposition

The task then picks up linting findings as work items alongside new clips. The sensor observes wiki health; the task acts on it. Clean separation.

This mirrors how the concern system works: the concern evaluation lens periodically assesses what matters; the task system acts on what's surfaced. The linting sensor is essentially a concern evaluation applied to a specific artifact rather than to Bruce's life.

## What's distinctive about doing this in Jill

A standalone script could do most of the mechanical work — watch for clips, call an LLM to write articles, maintain an index. What Jill adds:

**Priority through concerns.** Jill knows what Bruce currently cares about and can allocate curation effort accordingly. If Bruce is deep in a research push on RL, the RL knowledge base gets more compilation and linting cycles than dormant topics.

**Quality through envisioning.** The envision-then-execute pattern produces better articles than a single-shot "summarize this clip" prompt because Jill forms expectations about what the output should contain before generating it.

**Learning through similar-plans.** Jill gets better at compilation over time as her plan library accumulates successful patterns. A script starts from scratch every time.

**Integration through the user model.** Eventually, Jill's model of what Bruce knows, what confuses him, and what he's currently thinking about can inform how articles are written and what connections are surfaced. The knowledge base adapts to the person, not just the topic.

**Continuity through the task system.** The knowledge base is one of Jill's persistent commitments. She tracks what she's done, what's pending, what's broken. She doesn't need to be told "go process new clips" — she does it because it's her job.

## Implementation sequence (suggested)

1. **Obsidian write tool** — extend the existing tool with create/update/list operations. This is prerequisite plumbing.
2. **Large-page Collection decomposition** — build into the Obsidian read path so large pages come back as navigable Collections. This is needed before Jill can work with compiled wiki pages that grow beyond context limits.
3. **Clip-watcher sensor** — code-type sensor, inform disposition, watches a configured clippings directory.
4. **First task instance** — "Maintain knowledge base: [topic]" with a real topic Bruce is actively researching. Manual establishment, just to prove the compilation cycle works.
5. **Linting sensor** — plan-type sensor, periodic wiki health assessment.
6. **Q&A output filing** — extend chat-response to optionally write answers back to the knowledge base.
