# Metadata Is a Relation, Not a Note

If you're building a simple knowledge system — notes, collections, an LLM that manipulates them via tool calls — you will eventually hit the metadata problem. I want to record that this note is a summary of that note. Or that these two notes are about related papers. Or that this note supersedes an older one.

The obvious move: metadata is just a note *about* a note. Reify it. Everything is a Note, the ontology stays uniform, life is beautiful.

Except now a Note needs a way to refer to another Note. And that reference — where does it live? It's a field inside the first Note. Which makes it... metadata. About the Note. Which is what we were trying to extract in the first place. You're in a loop.

## The fix is a type distinction

Stop asking Notes to do double duty. You don't need a cleverer Note. You need a new primitive.

Three data classes:

- **Note** — an individual unit of content
- **Collection** — a set of Notes (or other Collections)
- **Relation** — a typed, directed connection between two objects (Note or Collection)

A Relation is not a Note with a `ref` field bolted on. It's a fundamentally different kind of thing. It *connects* objects rather than *being* one. The metadata circularity dissolves because you've stopped overloading a single data class.

Metadata is now just one use of Relation: a Relation where one Note is *about* another. But the same primitive handles "contradicts," "elaborates," "supersedes," "is related to" — any asymmetric, typed connection you need.

## What this looks like concretely

A Relation is a record with three fields:

```
{
  "source_id": "note_47",
  "target_id": "note_12",
  "type": "summarizes"
}
```

That's it. You store these in a flat list alongside your Notes and Collections. You look things up by filtering on any of the three fields. No graph database, no query language, no schema migration.

## Why this matters for LLM tooling

If you're building a system where an LLM manipulates knowledge objects via tool calls, the weight of your data model is a hard constraint. A mid-tier coding LLM can reliably emit a JSON object with three fields. It cannot reliably compose a Cypher query or manage a graph database API.

The property graph world (Neo4j, etc.) solved the typed-directed-edge problem long ago. But that machinery is drastically overbuilt for a system where each operation is a single tool call and the "database" is a JSON file. The insight from graph databases worth keeping: a typed, directed edge between nodes is the right primitive. The implementation they suggest — throw it away.

Keep it dumb. Three fields. Flat list. The LLM can handle it, you can inspect it by eye, and the metadata problem is solved without the metadata circularity.

## The ontological takeaway

The mistake was wanting everything to be a Note. Uniform ontologies are elegant until they force a single class to represent both content and the relationships between content. Once you let yourself have a Relation as a first-class citizen, the system gets simpler, not more complex. You moved from a propositional data model to a relational one — and the cost was one small data class with three fields.
