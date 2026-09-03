// The client page: one websocket, two panes. No framework.
(function () {
  const token = new URLSearchParams(location.search).get("token") || "";
  const $ = (id) => document.getElementById(id);
  const messages = $("messages"), doc = $("doc"), evidence = $("evidence"), text = $("text"), send = $("send");
  let kind = null, ws = null, thinking = false;
  let lastForm = null;          // the form as last rendered, to mark what an exchange changed
  let findings = {};            // post: every finding keyed "<source>#<id>"
  let sources = [];             // post: the claim sources, for a bare "claim N"
  let lastReply = "";           // post: the agent's last words, so a reload reopens what they named

  function esc(s) {
    return String(s == null ? "" : s).replace(/[&<>"]/g, (c) => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
  }

  // ---- a small markdown renderer for the agent's replies ----
  // Paragraphs, headings, bullet and numbered lists, fenced code, inline code,
  // bold and italic. Everything is escaped first; nothing else is HTML.
  function inline(s) {
    return esc(s)
      .replace(/`([^`]+)`/g, "<code>$1</code>")
      .replace(/\*\*([^*]+)\*\*/g, "<strong>$1</strong>")
      .replace(/(^|[^*])\*([^*\n]+)\*(?!\*)/g, "$1<em>$2</em>");
  }
  function md(s) {
    const out = [];
    const lines = String(s || "").replace(/\r/g, "").split("\n");
    let i = 0;
    while (i < lines.length) {
      const l = lines[i];
      if (/^```/.test(l)) {
        const buf = []; i++;
        while (i < lines.length && !/^```/.test(lines[i])) buf.push(lines[i++]);
        i++; out.push("<pre>" + esc(buf.join("\n")) + "</pre>"); continue;
      }
      const h = /^(#{1,3})\s+(.*)$/.exec(l);
      if (h) { out.push("<h" + h[1].length + ">" + inline(h[2]) + "</h" + h[1].length + ">"); i++; continue; }
      if (/^\s*[-*]\s+/.test(l)) {
        const items = [];
        while (i < lines.length && /^\s*[-*]\s+/.test(lines[i])) items.push(lines[i++].replace(/^\s*[-*]\s+/, ""));
        out.push("<ul>" + items.map((t) => "<li>" + inline(t) + "</li>").join("") + "</ul>"); continue;
      }
      if (/^\s*\d+[.)]\s+/.test(l)) {
        const items = [];
        while (i < lines.length && /^\s*\d+[.)]\s+/.test(lines[i])) items.push(lines[i++].replace(/^\s*\d+[.)]\s+/, ""));
        out.push("<ol>" + items.map((t) => "<li>" + inline(t) + "</li>").join("") + "</ol>"); continue;
      }
      if (!l.trim()) { i++; continue; }
      const buf = [];
      while (i < lines.length && lines[i].trim() && !/^(```|#{1,3}\s|\s*[-*]\s|\s*\d+[.)]\s)/.test(lines[i])) buf.push(lines[i++]);
      out.push("<p>" + buf.map(inline).join("<br>") + "</p>");
    }
    return out.join("");
  }

  function addMessage(who, body, cls) {
    const el = document.createElement("div");
    el.className = "msg " + (cls || who);
    const label = '<span class="who">' + esc(who === "client" ? "you" : who) + "</span>";
    el.innerHTML = label + (who === "agent" ? md(body) : esc(body));
    messages.appendChild(el);
    messages.scrollTop = messages.scrollHeight;
  }
  function setThinking(on) {
    thinking = on;
    $("thinking").hidden = !on;
    send.disabled = on;
  }

  // ---- the intake form ----
  function renderForm(d) {
    const form = d.form || {}, prev = lastForm;
    let h = '<div class="form"><h2>Intake form</h2><div class="ledger">' + esc(d.ledger || "") + "</div>";
    let firstChanged = null;
    for (const [group, fields] of Object.entries(d.slots || {})) {
      h += '<div class="group"><h3>' + esc(group) + "</h3>";
      for (const f of fields) {
        const v = (form[group] || {})[f];
        const empty = !(v && String(v).trim());
        const was = prev ? ((prev[group] || {})[f] || "") : (v || "");
        const changed = prev && String(was) !== String(v || "");
        const id = "f-" + group + "-" + f;
        if (changed && !firstChanged) firstChanged = id;
        h += '<div id="' + id + '" class="field' + (empty ? " empty" : "") + (changed ? " changed" : "")
          + '"><div class="k">' + esc(f) + '</div><div class="v">' + (empty ? "not yet" : esc(v)) + "</div></div>";
      }
      h += "</div>";
    }
    if ((form.open_questions || []).length) {
      h += '<div class="group"><h3>open questions</h3><ul class="list">'
        + form.open_questions.map((q) => "<li>" + esc(q) + "</li>").join("") + "</ul></div>";
    }
    h += "</div>";
    doc.innerHTML = h;
    lastForm = JSON.parse(JSON.stringify(form));
    if (firstChanged) { const el = $(firstChanged); if (el) el.scrollIntoView({block: "center", behavior: "smooth"}); }
    $("uploadLabel").hidden = false;
    if (d.uploads_dir) { $("uploadsHint").hidden = false; $("uploadsHint").textContent = "files for the seller go to: " + d.uploads_dir; }
  }

  // ---- the report, and the evidence pane ----
  function renderReport(d) {
    findings = d.findings || {};
    sources = Array.from(new Set(Object.values(findings).map((f) => f.claim_source)));
    doc.innerHTML = '<div class="banner">' + esc(d.banner || "") + '</div><div class="report">'
      + (d.html || "<p><i>No report has been written for this run yet.</i></p>") + "</div>";
    // Every finding heading reads "<source>, claim <id> — …"; a click shows its evidence.
    for (const h3 of doc.querySelectorAll(".report h3")) {
      const m = /^(\S+), claim (\d+)/.exec(h3.textContent.trim());
      if (!m) continue;
      h3.dataset.key = m[1] + "#" + m[2];
      h3.addEventListener("click", () => showFinding(h3.dataset.key, false));
    }
  }
  function headingFor(key) {
    return Array.from(doc.querySelectorAll(".report h3")).find((h) => h.dataset.key === key);
  }
  function showFinding(key, scroll) {
    const f = findings[key];
    if (!f) return;
    for (const s of doc.querySelectorAll(".finding.named")) s.classList.remove("named");
    const h3 = headingFor(key);
    if (h3) {
      const box = h3.closest(".finding"); if (box) box.classList.add("named");
      if (scroll) h3.scrollIntoView({block: "start", behavior: "smooth"});
    }
    const adverse = ["contradicted", "partial", "real_with_caveat"].includes(f.verdict);
    let h = '<div class="head"><h4>' + esc(f.claim_source) + ", claim " + esc(f.claim_id)
      + ' <span class="verdict' + (adverse ? " adverse" : "") + '">' + esc(f.verdict)
      + (f.review ? " · review: " + esc(f.review) : "") + "</span></h4>"
      + '<button class="close" id="evClose" title="close">×</button></div>';
    h += "<blockquote>" + esc(f.quote) + " (" + esc(f.claim_source) + ", lines " + esc((f.lines || []).join("–")) + ")</blockquote>";
    if (f.gap) h += '<p class="gap"><b>The gap:</b> ' + esc(f.gap) + "</p>";
    if (f.unresolved_because) h += '<p class="gap"><b>Why unsettled:</b> ' + esc(f.unresolved_because) + "</p>";
    for (const e of f.evidence || []) {
      if (e.form === "citation") {
        h += '<div class="item"><div class="where">' + esc(e.document) + ":" + esc((e.lines || []).join("–")) + "</div>"
          + "<pre>" + esc(e.quote) + "</pre>" + '<div class="shows">' + esc(e.shows) + "</div></div>";
      } else if (e.form === "search") {
        h += '<div class="item"><div class="where">search (' + esc(e.kind) + "): " + esc(e.performed) + "</div>"
          + '<div class="shows">' + esc(e.result) + "</div></div>";
      } else {
        h += '<div class="item"><div class="where">derived</div><div class="shows">' + esc(e.derivation) + "<br>" + esc(e.consequence) + "</div></div>";
      }
    }
    evidence.innerHTML = h;
    evidence.hidden = false;
    $("evClose").addEventListener("click", () => { evidence.hidden = true; });
  }
  // A reply that names a finding — "README.md, claim 20", "claim 20" — opens it.
  function findingsNamed(replyText) {
    const keys = [];
    const re = /(?:([\w.\-]+\.\w+),?\s+)?claim(?:_id)?\s+#?(\d+)/gi;
    let m;
    while ((m = re.exec(replyText)) !== null) {
      const src = m[1] || (sources.length === 1 ? sources[0] : null);
      if (!src) continue;
      const key = src + "#" + m[2];
      if (findings[key] && !keys.includes(key)) keys.push(key);
    }
    return keys;
  }

  function renderDocument(d) {
    $("engagement").textContent = d.engagement || "engagement";
    document.title = (d.engagement || "engagement") + (d.kind === "form" ? " — intake" : " — report");
    if (d.kind === "form") renderForm(d); else renderReport(d);
  }

  // ---- the websocket ----
  function connect() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    ws = new WebSocket(proto + "://" + location.host + "/ws?token=" + encodeURIComponent(token));
    ws.onopen = () => { $("conn").textContent = "connected"; };
    ws.onclose = () => { $("conn").textContent = "disconnected — retrying"; setTimeout(connect, 2000); };
    ws.onmessage = (ev) => {
      const m = JSON.parse(ev.data);
      if (m.type === "history") {
        kind = m.kind; $("mode").textContent = kind === "intake" ? "intake" : "report";
        messages.innerHTML = "";
        for (const t of m.history || []) { addMessage(t.who, t.text); if (t.who === "agent") lastReply = t.text; }
      } else if (m.type === "say") {
        const who = m.who === "client" ? "client" : "agent";
        addMessage(who, m.text);
        if (who === "agent" && kind === "post") {
          lastReply = m.text;
          const named = findingsNamed(m.text);
          if (named.length) showFinding(named[0], true);
        }
      } else if (m.type === "document") {
        renderDocument(m);
        if (m.kind === "report" && lastReply) {
          const named = findingsNamed(lastReply);
          if (named.length) showFinding(named[0], true);
        }
      } else if (m.type === "status") {
        setThinking(m.state === "thinking");
      } else if (m.type === "error") {
        addMessage("error", m.text, "error");
      }
    };
  }

  // ---- composing ----
  function submit() {
    const t = text.value.trim();
    if (!t || thinking || !ws || ws.readyState !== 1) return;
    ws.send(JSON.stringify({type: "turn", text: t}));
    text.value = "";
  }
  $("compose").addEventListener("submit", (e) => { e.preventDefault(); submit(); });
  text.addEventListener("keydown", (e) => { if (e.key === "Enter" && !e.shiftKey) { e.preventDefault(); submit(); } });
  $("file").addEventListener("change", async (e) => {
    const f = e.target.files[0];
    if (!f) return;
    const body = new FormData(); body.append("file", f);
    const r = await fetch("/api/upload?token=" + encodeURIComponent(token), {method: "POST", body});
    if (!r.ok) addMessage("error", "upload failed: " + r.status, "error");
    e.target.value = "";
  });

  connect();
})();
