// The claim surface: every claim of every source, the comments on each.
// A client reads and comments. The practice edits a draft, drops and adds
// claims, saves, and freezes; a frozen source is read-only for everyone.
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
      }));   // implied_by, property and approved_by ride along from orig
    }
    return out;
  }

  function renderSource(src, editable) {
    const ed = editable && !src.frozen;
    const byClaim = {};
    for (const c of src.comments || []) (byClaim[String(c.claim_id)] = byClaim[String(c.claim_id)] || []).push(c);
    let h = '<div class="source"><h2>' + esc(src.source) + ' <span class="tag' + (src.frozen ? " current" : "") + '">'
      + (src.frozen ? "frozen" : src.origin === "draft" ? "draft, not frozen" : src.origin === "enumeration" ? "as enumerated (" + esc(src.run) + ")" : "not enumerated yet") + "</span></h2>";
    if (!src.claims.length) { h += '<p class="muted">No claims yet.</p></div>'; return h; }
    h += '<table class="claims"><tr><th>#</th><th>lines</th><th>quote</th><th>statement</th><th>comments</th>' + (ed ? "<th></th>" : "") + "</tr>";
    for (const c of src.claims) {
      const cm = byClaim[String(c.id)] || [];
      h += '<tr data-src="' + esc(src.slug) + '" data-id="' + esc(c.id) + '">'
        + '<td class="id">' + esc(c.id) + (c.about === "seller" ? '<div class="muted">seller</div>' : c.about === "document" ? '<div class="muted">document</div>' : "")
          + (c.implied_by != null ? '<div class="muted">implied by ' + esc(c.implied_by) + "</div>" : "") + "</td>"
        + '<td class="lines mono"' + (ed ? ' contenteditable="true"' : "") + ">" + esc((c.lines || []).join("–")) + "</td>"
        + '<td class="quote"' + (ed ? ' contenteditable="true"' : "") + ">" + esc(c.quote) + "</td>"
        + '<td class="statement"' + (ed ? ' contenteditable="true"' : "") + ">" + esc(c.statement) + "</td>"
        + '<td class="comments">' + cm.map((x) => '<div class="c"><span class="by">' + esc(x.by) + "</span> " + esc(x.text) + "</div>").join("")
        + '<form class="comment" data-src="' + esc(src.source) + '" data-id="' + esc(c.id) + '"><input placeholder="comment"><button>add</button></form></td>'
        + (ed ? '<td><button class="quiet drop" title="leave this claim out">drop</button>'
              + (c.implied_by == null ? '<button class="quiet decompose" title="ask for the testable properties a reasonable buyer would take this claim to assert">decompose</button>' : "") + "</td>" : "")
        + "</tr>";
    }
    h += "</table>";
    if (ed) h += '<div class="actions"><button class="quiet addClaim" data-slug="' + esc(src.slug) + '">add a claim</button> '
      + '<button class="save" data-source="' + esc(src.source) + '">Save draft</button> '
      + '<button class="freeze" data-source="' + esc(src.source) + '">Freeze</button></div>';
    return h + "</div>";
  }

  function render() {
    $("engagement").textContent = data.name;
    document.title = data.name + " — claim surface";
    $("intro").textContent = data.editable
      ? "Edit any cell, drop what is not a claim, add what is missing, save the draft, then freeze. The client's comments are beside each claim."
      : "These are the claims the review will test, as enumerated from the documents you named. Comment on any claim that is wrong, missing or beside the point; the practice reads every comment before freezing the list.";
    $("sources").innerHTML = data.sources.map((s) => renderSource(s, data.editable)).join("");
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
        if (!confirm("Freeze the surface for " + src.source + "? The review tests exactly these claims.")) return;
        const saved = await api("api/draft", {source: src.source, claims: claimsOf(src)});
        if (!saved) return;
        const j = await api("api/freeze", {source: src.source});
        if (j) { data.sources = data.sources.map((s) => s.source === j.source ? j : s); render(); }
      });
    }
  }

  $("back").href = (location.pathname.startsWith("/p/") ? "/p/" : location.pathname.replace(/surface\/$/, "")) + qs;
  (async () => { data = await api("api"); if (data) render(); })();
})();
