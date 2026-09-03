// The client page: one websocket, two panes. No framework.
(function () {
  const token = new URLSearchParams(location.search).get("token") || "";
  const $ = (id) => document.getElementById(id);
  const messages = $("messages"), doc = $("doc"), text = $("text"), send = $("send");
  let kind = null, ws = null, thinking = false;

  function esc(s) {
    return String(s == null ? "" : s).replace(/[&<>"]/g, (c) => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
  }
  function addMessage(who, body, cls) {
    const el = document.createElement("div");
    el.className = "msg " + (cls || who);
    el.innerHTML = '<span class="who">' + esc(who === "client" ? "you" : who) + "</span>" + esc(body);
    messages.appendChild(el);
    messages.scrollTop = messages.scrollHeight;
  }
  function setThinking(on) {
    thinking = on;
    $("thinking").hidden = !on;
    send.disabled = on;
  }

  // ---- the document pane ----
  function renderForm(d) {
    const form = d.form || {}, check = d.check || {};
    let h = '<div class="form"><h2>Intake form</h2><div class="ledger">' + esc(d.ledger || "") + "</div>";
    for (const [group, fields] of Object.entries(d.slots || {})) {
      h += '<div class="group"><h3>' + esc(group) + "</h3>";
      for (const f of fields) {
        const v = (form[group] || {})[f];
        const empty = !(v && String(v).trim());
        h += '<div class="field' + (empty ? " empty" : "") + '"><div class="k">' + esc(f) + '</div><div class="v">'
          + (empty ? "not yet" : esc(v)) + "</div></div>";
      }
      h += "</div>";
    }
    if ((form.open_questions || []).length) {
      h += '<div class="group"><h3>open questions</h3><ul class="list">'
        + form.open_questions.map((q) => "<li>" + esc(q) + "</li>").join("") + "</ul></div>";
    }
    h += "</div>";
    doc.innerHTML = h;
    $("uploadLabel").hidden = false;
    if (d.uploads_dir) { $("uploadsHint").hidden = false; $("uploadsHint").textContent = "files for the seller go to: " + d.uploads_dir; }
  }
  function renderReport(d) {
    doc.innerHTML = '<div class="banner">' + esc(d.banner || "") + '</div><div class="report">'
      + (d.html || "<p><i>No report has been written for this run yet.</i></p>") + "</div>";
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
        for (const t of m.history || []) addMessage(t.who, t.text);
      } else if (m.type === "say") {
        addMessage(m.who === "client" ? "client" : "agent", m.text);
      } else if (m.type === "document") {
        renderDocument(m);
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
