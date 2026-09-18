// The claim surface: every claim of every source, the comments on each.
// A client reads and comments. The practice edits a draft, drops and adds
// claims, saves, and freezes; a frozen source is read-only for everyone.
// Above the claims: what the buyer relies on the offering for, which the
// practice corrects. Beside each claim: its tier. Tier 1 is tested; tiers 2
// and 3 are listed and not tested. The practice changes any tier before the
// freeze, and a tier it set is marked so a later rating leaves it alone.
(function () {
  const $ = (id) => document.getElementById(id);
  const esc = window.mdlib.esc;
  const qs = location.search || "";
  let data = null;

  async function api(path, body) {
    const r = await fetch(path + qs, body ? {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)} : {});
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { $("msg").textContent = j.detail || ("error " + r.status); return null; }
    $("msg").textContent = "";
    return j;
  }

  function claimsOf(src) {
    // The practice's edits live in the table; read them back into claims.json shape.
    const rows = document.querySelectorAll('tr[data-src="' + CSS.escape(src.slug) + '"]');
    const out = [];
    for (const tr of rows) {
      if (tr.dataset.dropped === "1") continue;
      const id = Number(tr.dataset.id);
      const lines = tr.querySelector(".lines").textContent.trim().split(/[-–]/).map((x) => Number(x.trim()));
      const orig = src.claims.find((c) => c.id === id) || {};
      out.push(Object.assign({}, orig, {
        id, lines: lines.length === 2 && lines.every((n) => !isNaN(n)) ? lines : (orig.lines || [0, 0]),
        quote: tr.querySelector(".quote").textContent.trim(),
        statement: tr.querySelector(".statement").textContent.trim(),
      }, tierOf(tr, orig)));   // implied_by, property and approved_by ride along from orig
    }
    return out;
  }

  function tierOf(tr, orig) {
    // The tier as the row now shows it. A tier the practice chose, unlike the
    // one proposed, is marked `tier_by`; the reason is the cell's text.
    const sel = tr.querySelector("select.tier");
    if (!sel) return {};
    const tier = sel.value ? Number(sel.value) : null;
    const basis = tr.querySelector(".basis").textContent.trim();
    const out = {tier_basis: basis};
    if (tier === null) { out.tier = undefined; out.tier_by = undefined; return out; }
    out.tier = tier;
    if (orig.tier_by || tier !== orig.tier || basis !== (orig.tier_basis || "")) out.tier_by = "practice";
    return out;
  }

  function tierCell(c, ed) {
    const basis = '<div class="basis muted"' + (ed ? ' contenteditable="true"' : "") + ">" + esc(c.tier_basis || "") + "</div>";
    const by = c.tier_by ? '<div class="muted">set by the practice</div>' : "";
    if (!ed) return '<td class="tier">' + (c.tier ? "tier " + esc(c.tier) : "") + by + basis + "</td>";
    return '<td class="tier"><select class="tier">' + ["", 1, 2, 3].map((v) =>
      '<option value="' + v + '"' + (String(c.tier || "") === String(v) ? " selected" : "") + ">" + (v === "" ? "not rated" : "tier " + v) + "</option>").join("")
      + "</select>" + by + basis + "</td>";
  }

  function renderReliance(rel, ed) {
    if (!rel || !(rel.items || []).length) return "";
    const kinds = ["depends", "uses", "does_not_use"], sources = ["buyer", "inference"];
    const pick = (cls, vals, cur) => ed
      ? '<select class="' + cls + '">' + vals.map((v) => '<option' + (v === cur ? " selected" : "") + ">" + esc(v) + "</option>").join("") + "</select>"
      : esc(cur);
    const ce = ed ? ' contenteditable="true"' : "";
    let h = '<div class="source" id="relianceBox"><h2>What the buyer relies on <span class="tag">'
      + (rel.corrected_by ? "corrected by " + esc(rel.corrected_by) : "as written by the practice's process, not yet corrected") + "</span></h2>"
      + '<p class="muted">' + (ed ? "Every tier below was rated against this statement. Correct it where it is wrong, save it, and rate the claims again with the tiers command on the practice page; tiers you set yourself are kept."
        : "The practice's account of what you rely on this software for. The claims below were rated against it. Comment on any claim, or tell the practice, where it is wrong.") + "</p>"
      + '<p id="relUse"' + ce + ">" + esc(rel.use || "") + "</p>"
      + '<table class="claims" id="relItems"><tr><th>item</th><th>reliance</th><th>if it failed</th><th>rests on</th><th>the buyer\'s words</th></tr>';
    for (const x of rel.items) {
      h += "<tr><td class=\"item\"" + ce + ">" + esc(x.item) + "</td><td>" + pick("rel", kinds, x.reliance) + "</td>"
        + '<td class="fail"' + ce + ">" + esc(x.if_it_failed) + "</td><td>" + pick("src", sources, x.source) + "</td>"
        + '<td class="words quote"' + ce + ">" + esc(x.buyer_words) + "</td></tr>";
    }
    h += "</table>";
    if (ed) h += '<div class="actions"><button class="quiet" id="relAdd">add an item</button> <button id="relSave">Save the statement</button></div>';
    return h + "</div>";
  }

  function relianceOf() {
    const items = [];
    for (const tr of document.querySelectorAll("#relItems tr")) {
      const item = tr.querySelector(".item");
      if (!item) continue;
      items.push({item: item.textContent.trim(), reliance: tr.querySelector("select.rel").value,
        if_it_failed: tr.querySelector(".fail").textContent.trim(), source: tr.querySelector("select.src").value,
        buyer_words: tr.querySelector(".words").textContent.trim()});
    }
    return {use: $("relUse").textContent.trim(), items};
  }

  function renderSource(src, editable) {
    const ed = editable && !src.frozen;
    const byClaim = {};
    for (const c of src.comments || []) (byClaim[String(c.claim_id)] = byClaim[String(c.claim_id)] || []).push(c);
    let h = '<div class="source"><h2>' + esc(src.source) + ' <span class="tag' + (src.frozen ? " current" : "") + '">'
      + (src.frozen ? "frozen" : src.origin === "draft" ? "draft, not frozen" : src.origin === "enumeration" ? "as enumerated (" + esc(src.run) + ")" : "not enumerated yet") + "</span></h2>";
    if (!src.claims.length) { h += '<p class="muted">No claims yet.</p></div>'; return h; }
    h += '<table class="claims"><tr><th>#</th><th>lines</th><th>quote</th><th>statement</th><th>tier</th><th>comments</th>' + (ed ? "<th></th>" : "") + "</tr>";
    for (const c of src.claims) {
      const cm = byClaim[String(c.id)] || [];
      // A claim the duplicates pass marked starts left out: the earlier claim is
      // the one tested. The statement it was matched with is shown under its
      // own, and "keep" brings it back.
      const dup = !!c.same_as && ed;
      h += '<tr data-src="' + esc(src.slug) + '" data-id="' + esc(c.id) + '"' + (dup ? ' data-dropped="1" class="dropped"' : (c.tier === 2 || c.tier === 3) ? ' class="untested"' : "") + ">"
        + '<td class="id">' + esc(c.id) + (c.about === "seller" ? '<div class="muted">seller</div>' : c.about === "document" ? '<div class="muted">document</div>' : "")
          + (c.implied_by != null ? '<div class="muted">implied by ' + esc(c.implied_by) + (c.property ? ": " + esc(c.property) : "") + "</div>" : "")
          + (c.same_as ? '<div class="muted">same as ' + esc(c.same_as.source) + " " + esc(c.same_as.id) + "</div>" : "")
          + (c.within ? '<div class="muted">within ' + esc(c.within.source) + " " + esc(c.within.id) + "</div>" : "")
          + ((c.declined || []).length ? '<div class="declined">not decomposed: ' + c.declined.map((d) => esc(d.text) + " (" + esc(d.why) + ")").join("; ") + "</div>" : "") + "</td>"
        + '<td class="lines mono"' + (ed ? ' contenteditable="true"' : "") + ">" + esc((c.lines || []).join("–")) + "</td>"
        + '<td class="quote"' + (ed ? ' contenteditable="true"' : "") + ">" + esc(c.quote) + "</td>"
        + '<td class="statement"' + (ed ? ' contenteditable="true"' : "") + ">" + esc(c.statement) + "</td>"
        // (the declined parts of a decomposed claim are shown under its id, below)
        + tierCell(c, ed)
        + '<td class="comments">' + cm.map((x) => '<div class="c"><span class="by">' + esc(x.by) + "</span> " + esc(x.text) + "</div>").join("")
        + '<form class="comment" data-src="' + esc(src.source) + '" data-id="' + esc(c.id) + '"><input placeholder="comment"><button>add</button></form></td>'
        + (ed ? '<td><button class="quiet drop" title="leave this claim out">' + (dup ? "keep" : "drop") + "</button>"
              + (c.same_as ? '<div class="declined">tested as ' + esc(c.same_as.source) + " " + esc(c.same_as.id) + ": " + esc(c.same_as.statement) + "</div>" : "")
              // A narrower claim stays in the audit: its verdict is not yet read from the wider one's.
              + (c.within ? '<div class="declined">covered by ' + esc(c.within.source) + " " + esc(c.within.id) + ": " + esc(c.within.statement) + "</div>" : "")
              + (c.implied_by == null ? '<button class="quiet decompose" title="ask for the testable properties a reasonable buyer would take this claim to assert">decompose</button>' : "") + "</td>" : "")
        + "</tr>";
    }
    h += "</table>";
    if (ed) h += '<div class="actions"><button class="quiet addClaim" data-slug="' + esc(src.slug) + '">add a claim</button> '
      + '<button class="save" data-source="' + esc(src.source) + '">Save draft</button> '
      + '<button class="freeze" data-source="' + esc(src.source) + '">Freeze</button></div>';
    else if (editable && src.frozen) h += '<div class="actions"><button class="quiet unfreeze" data-source="' + esc(src.source)
      + '" title="make the surface editable again before a rerun">Unfreeze</button></div>';
    return h + "</div>";
  }

  function render() {
    $("engagement").textContent = data.name;
    document.title = data.name + " — claim surface";
    if (data.editable) {
      $("intro").innerHTML = "Edit any cell, drop what is not a claim, add what is missing, save the draft, then freeze. The client's comments are beside each claim. "
        + '<a href="/p/guidance/' + qs + '">Read the scrub guidance</a> before you start.';
    } else {
      $("intro").textContent = "These are the claims the review will test, as enumerated from the documents you named. Comment on any claim that is wrong, missing or beside the point; the practice reads every comment before freezing the list.";
    }
    $("sources").innerHTML = renderReliance(data.reliance, data.editable) + data.sources.map((s) => renderSource(s, data.editable)).join("");
    if ($("relSave")) {
      $("relSave").addEventListener("click", async () => {
        const j = await api("api/reliance", relianceOf());
        if (j) { data.reliance = j; render(); $("msg").textContent = "statement saved; rate the claims again to apply it"; }
      });
      $("relAdd").addEventListener("click", () => {
        data.reliance = relianceOf();
        data.reliance.items.push({item: "", reliance: "uses", if_it_failed: "", source: "inference", buyer_words: ""});
        render();
      });
    }
    for (const f of document.querySelectorAll("form.comment")) {
      f.addEventListener("submit", async (e) => {
        e.preventDefault();
        const text = f.querySelector("input").value.trim();
        if (!text) return;
        const j = await api("api/comment", {source: f.dataset.src, claim_id: Number(f.dataset.id), text});
        if (j) { data.sources = data.sources.map((s) => s.source === j.source ? j : s); render(); }
      });
    }
    for (const b of document.querySelectorAll("button.drop")) {
      b.addEventListener("click", () => { const tr = b.closest("tr"); tr.dataset.dropped = tr.dataset.dropped === "1" ? "0" : "1"; tr.classList.toggle("dropped"); b.textContent = tr.dataset.dropped === "1" ? "keep" : "drop"; });
    }
    for (const b of document.querySelectorAll("button.decompose")) {
      b.addEventListener("click", async () => {
        const tr = b.closest("tr");
        const src = data.sources.find((s) => s.slug === tr.dataset.src);
        const id = Number(tr.dataset.id);
        b.disabled = true; b.textContent = "asking…"; $("msg").textContent = "the agent is reading the claim";
        const j = await api("api/decompose", {source: src.source, claim_id: id, claims: claimsOf(src)});
        if (!j) { b.disabled = false; b.textContent = "decompose"; return; }
        // The proposals join the draft as rows the practice can edit or drop; save keeps them.
        const current = claimsOf(src);
        const parent = current.find((c) => c.id === id);
        if (parent) parent.declined = j.declined || [];        // the reasons stay with the claim, and are saved with the draft
        for (const row of j.subclaims) current.push(row);
        src.claims = current;
        render();
        const declined = (j.declined || []).map((d) => d.text + " — " + d.why).join("; ");
        $("msg").textContent = j.subclaims.length + " proposed" + (declined ? " · declined: " + declined : "") + " · edit, drop, then save the draft";
      });
    }
    for (const b of document.querySelectorAll("button.addClaim")) {
      b.addEventListener("click", () => {
        const src = data.sources.find((s) => s.slug === b.dataset.slug);
        const id = Math.max(0, ...src.claims.map((c) => c.id)) + 1;
        src.claims.push({id, lines: [0, 0], quote: "", statement: "", about: "target"});
        render();
      });
    }
    for (const b of document.querySelectorAll("button.save")) {
      b.addEventListener("click", async () => {
        const src = data.sources.find((s) => s.source === b.dataset.source);
        const j = await api("api/draft", {source: src.source, claims: claimsOf(src)});
        if (j) { data.sources = data.sources.map((s) => s.source === j.source ? j : s); render(); $("msg").textContent = "draft saved"; }
      });
    }
    for (const b of document.querySelectorAll("button.freeze")) {
      b.addEventListener("click", async () => {
        const src = data.sources.find((s) => s.source === b.dataset.source);
        const now = claimsOf(src), n = (k) => now.filter((c) => c.tier === k).length;
        const tested = now.length - n(2) - n(3);
        if (!confirm("Freeze the surface for " + src.source + "? The review tests " + tested + " claim(s): tier 1 and any not rated. "
          + "It lists and does not test " + n(2) + " in tier 2 and " + n(3) + " in tier 3.")) return;
        const saved = await api("api/draft", {source: src.source, claims: claimsOf(src)});
        if (!saved) return;
        const j = await api("api/freeze", {source: src.source});
        if (j) { data.sources = data.sources.map((s) => s.source === j.source ? j : s); render(); }
      });
    }
    for (const b of document.querySelectorAll("button.unfreeze")) {
      b.addEventListener("click", async () => {
        const src = data.sources.find((s) => s.source === b.dataset.source);
        if (!confirm("Unfreeze the surface for " + src.source + "? It becomes a draft again; freeze it before the rerun.")) return;
        const j = await api("api/unfreeze", {source: src.source});
        if (j) { data.sources = data.sources.map((s) => s.source === j.source ? j : s); render(); $("msg").textContent = "unfrozen; the frozen file is archived"; }
      });
    }
  }

  $("back").href = (location.pathname.startsWith("/p/") ? "/p/" : location.pathname.replace(/surface\/$/, "")) + qs;
  (async () => { data = await api("api"); if (data) render(); })();
})();
