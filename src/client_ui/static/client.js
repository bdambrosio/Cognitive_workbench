// The client page: one websocket, two panes. No framework.
(function () {
  const token = new URLSearchParams(location.search).get("token") || "";
  // The single-session page carries a token on the URL; the public demo
  // carries none and identifies the visitor by cookie; the site carries the
  // identity in a cookie or, locally, in ?as=. The page's query string is
  // passed along whole.
  const tq = token ? "?token=" + encodeURIComponent(token) : "";
  const $ = (id) => document.getElementById(id);
  const messages = $("messages"), doc = $("doc"), evidence = $("evidence"), text = $("text"), send = $("send");
  let kind = null, ws = null, thinking = false;
  let lastForm = null;          // the form as last rendered, to mark what an exchange changed
  let findings = {};            // post: every finding keyed "<source>#<id>"
  let sources = [];             // post: the claim sources, for a bare "claim N"
  let lastReply = "";           // post: the agent's last words, so a reload reopens what they named

  // The renderer is shared (md.js); this page also runs under the site's
  // engagement paths, so every address is relative to the page's own path.
  const esc = window.mdlib.esc, md = window.mdlib.render;
  const base = location.pathname.replace(/\/$/, "");
  const qs = location.search || tq;

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
    // A visible bubble in the conversation while the agent composes: the
    // header word alone was missed (Bruce, 2026-09-05).
    let w = document.getElementById("waiting");
    if (on && !w) {
      w = document.createElement("div"); w.id = "waiting"; w.className = "msg agent waiting";
      w.innerHTML = '<span class="who">agent</span><span class="dots"><i></i><i></i><i></i></span>';
      messages.appendChild(w); messages.scrollTop = messages.scrollHeight;
    } else if (!on && w) { w.remove(); }
  }

  // ---- the intake form ----
  function renderForm(d) {
    const form = d.form || {}, prev = lastForm;
    let h = '<div class="form"><h2>Intake form</h2><div class="ledger">' + esc(d.ledger || "") + '</div><div class="finish-hint" id="finishHint" hidden></div>';
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
    $("uploadsHint").hidden = false; $("uploadsHint").textContent = "A file you upload is attached to this engagement and read by the review.";
    // On the site the client finishes the intake from the page; the button
    // is enabled once every slot is filled and stays until the practice
    // has what it needs.
    const fb = $("finishBtn");
    if (fb && d.finish) {
      fb.hidden = false;
      fb.disabled = !d.finish.allowed || d.finish.done;
      fb.textContent = d.finish.done ? "Intake finished" : "Finish intake";
      fb.title = d.finish.allowed ? "" : "Answer at least one question first.";
      fb.dataset.empty = JSON.stringify(d.finish.empty || {});
      const ready = d.finish.allowed && !d.finish.done;
      const full = ready && !Object.keys(d.finish.empty || {}).length;
      fb.classList.toggle("ready", full);
      // The form says where the button is; a client asking the agent how to
      // finish was the first live question (2026-09-05).
      const hint = $("finishHint");
      if (hint) {
        hint.hidden = d.finish.done;
        hint.textContent = d.finish.done ? "" : full
          ? "Every field is filled. When you have nothing to add, press \u201cFinish intake\u201d below the conversation."
          : "When you have said what you need to, press \u201cFinish intake\u201d below the conversation; empty fields are read as not stated.";
      }
    }
  }
  async function finishIntake() {
    const fb = $("finishBtn");
    const empty = JSON.parse(fb.dataset.empty || "{}");
    const names = Object.entries(empty).flatMap(([slot, fs]) => fs.map((f) => slot + "." + f));
    if (names.length && !confirm("Finish the intake with " + names.length + " field" + (names.length > 1 ? "s" : "") + " still empty?\n\n" + names.join("\n") + "\n\nEmpty fields are read as 'not stated'.")) return;
    fb.disabled = true;
    const r = await fetch(base + "/api/finish" + qs, {method: "POST"});
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { addMessage("error", j.detail || ("finish failed: " + r.status), "error"); fb.disabled = false; return; }
    addMessage("agent", "The intake is finished. " + ((j.next || {}).text || ""));
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
    linkReferences();
    buildOutline();
  }
  // Every "<source>, claim <id>" in the report's prose, lists and appendix
  // rows becomes a link to that finding: the executive summary's list of
  // material findings and the appendix table are the finding-level outline.
  function linkReferences() {
    const re = /([\w.\-]+\.\w+), claim (\d+)/g;
    for (const el of doc.querySelectorAll(".report p, .report li, .report td")) {
      if (el.closest("h3")) continue;
      const walker = document.createTreeWalker(el, NodeFilter.SHOW_TEXT);
      const nodes = [];
      while (walker.nextNode()) nodes.push(walker.currentNode);
      for (const node of nodes) {
        const t = node.nodeValue;
        if (!re.test(t)) { re.lastIndex = 0; continue; }
        re.lastIndex = 0;
        const frag = document.createDocumentFragment();
        let last = 0, m;
        while ((m = re.exec(t)) !== null) {
          const key = m[1] + "#" + m[2];
          if (!findings[key]) continue;
          frag.appendChild(document.createTextNode(t.slice(last, m.index)));
          const a = document.createElement("a");
          a.className = "ref"; a.textContent = m[0]; a.href = "#";
          a.addEventListener("click", (ev) => { ev.preventDefault(); showFinding(key, true); });
          frag.appendChild(a);
          last = m.index + m[0].length;
        }
        frag.appendChild(document.createTextNode(t.slice(last)));
        node.parentNode.replaceChild(frag, node);
      }
    }
    // The appendix table — the one whose second column is "id": the id
    // cell of each row is the link. The scope table also has a numeric
    // second column (the claim count) and is not a list of claims.
    for (const tr of doc.querySelectorAll(".report table tbody tr")) {
      const th = tr.closest("table").querySelectorAll("thead th");
      if (th.length < 2 || th[1].textContent.trim() !== "id") continue;
      const tds = tr.querySelectorAll("td");
      if (tds.length < 2 || !/^\d+$/.test(tds[1].textContent.trim())) continue;
      const key = tds[0].textContent.trim() + "#" + tds[1].textContent.trim();
      if (!findings[key]) continue;
      const a = document.createElement("a");
      a.className = "ref"; a.textContent = tds[1].textContent.trim(); a.href = "#";
      a.addEventListener("click", (ev) => { ev.preventDefault(); showFinding(key, true); });
      tds[1].textContent = ""; tds[1].appendChild(a);
    }
  }
  // A bar of the report's sections above the scrolling document, the
  // current one marked as the reader scrolls. Built from the headings, so
  // the renderer is unchanged; sits outside #doc so it never scrolls away.
  function buildOutline() {
    const old = $("toc"); if (old) old.remove();
    const heads = Array.from(doc.querySelectorAll(".report h2"));
    if (heads.length < 2) return;
    const toc = document.createElement("nav");
    toc.id = "toc";
    const links = heads.map((h2) => {
      const a = document.createElement("a");
      a.textContent = h2.textContent.split(" — ")[0].trim(); a.href = "#";
      a.title = h2.textContent.trim();
      a.addEventListener("click", (ev) => { ev.preventDefault(); h2.scrollIntoView({block: "start", behavior: "smooth"}); });
      toc.appendChild(a);
      return a;
    });
    doc.parentNode.insertBefore(toc, doc);
    const mark = () => {
      const top = doc.getBoundingClientRect().top + 8;
      let cur = 0;
      heads.forEach((h2, i) => { if (h2.getBoundingClientRect().top <= top + 40) cur = i; });
      links.forEach((a, i) => a.classList.toggle("current", i === cur));
      const c = links[cur];
      if (c && (c.offsetLeft < toc.scrollLeft || c.offsetLeft + c.offsetWidth > toc.scrollLeft + toc.clientWidth))
        toc.scrollTo({left: c.offsetLeft - 20, behavior: "smooth"});
    };
    doc.addEventListener("scroll", mark, {passive: true});
    mark();
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
    ws = new WebSocket(proto + "://" + location.host + base + "/ws" + qs);
    ws.onopen = () => { $("conn").textContent = "connected"; };
    ws.onclose = (ev) => {
      // 1011 is the server saying the conversation cannot start; the error
      // message before it says why. Retrying would only repeat it.
      if (ev.code === 1011) { $("conn").textContent = "not started"; return; }
      $("conn").textContent = "disconnected — retrying"; setTimeout(connect, 2000);
    };
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
  if ($("finishBtn")) $("finishBtn").addEventListener("click", finishIntake);
  text.addEventListener("keydown", (e) => { if (e.key === "Enter" && !e.shiftKey) { e.preventDefault(); submit(); } });
  $("file").addEventListener("change", async (e) => {
    const f = e.target.files[0];
    if (!f) return;
    const body = new FormData(); body.append("file", f);
    const r = await fetch(base + "/api/upload" + qs, {method: "POST", body});
    if (!r.ok) addMessage("error", "upload failed: " + r.status, "error");
    e.target.value = "";
  });

  connect();
})();
