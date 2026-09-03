// The practice page: engagements, their intakes and runs, the current choices, the commands.
(function () {
  const token = new URLSearchParams(location.search).get("token") || "";
  const $ = (id) => document.getElementById(id);
  let data = [], selected = null;
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"]/g, (c) => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
  const q = (path) => path + (path.includes("?") ? "&" : "?") + "token=" + encodeURIComponent(token);

  async function api(path, body) {
    const r = await fetch(q(path), body ? {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)} : {});
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { say(j.detail || ("error " + r.status), true); return null; }
    say("");
    return j;
  }
  function say(t, warn) { const m = $("msg"); m.textContent = t; m.className = warn ? "warn" : ""; }

  function renderList() {
    let h = "";
    for (const e of data) {
      const ci = e.current_intake || "no intake";
      h += '<div class="eng' + (e.name === selected ? " selected" : "") + '" data-name="' + esc(e.name) + '">'
        + "<div>" + esc(e.name) + "</div><div class=\"sub\">" + esc(ci) + (e.has_target ? "" : " · no target") + "</div></div>";
    }
    h += '<div class="new"><div class="muted">New engagement</div>'
      + '<input id="newName" placeholder="name (letters, digits, . _ -)">'
      + '<input id="newClone" placeholder="clone from (git URL or local path), optional">'
      + '<button id="newBtn">Create</button></div>';
    $("list").innerHTML = h;
    for (const el of document.querySelectorAll(".eng")) el.addEventListener("click", () => { selected = el.dataset.name; render(); });
    $("newBtn").addEventListener("click", async () => {
      const name = $("newName").value.trim(), clone = $("newClone").value.trim();
      if (!name) return;
      const j = await api("/api/engagements", {name, clone: clone || null});
      if (j) { data = j; selected = name; render(); }
    });
  }

  function tags(x) {
    return (x.current ? '<span class="tag current">current</span>' : "")
      + (x.cancelled ? '<span class="tag cancelled">cancelled</span>' : "")
      + (x.finished === false ? '<span class="tag">not finished</span>' : "")
      + (x.report ? '<span class="tag">report</span>' : "")
      + (x.on_disk === false ? '<span class="tag">pinned only</span>' : "");
  }
  function runRows(e, runs) {
    if (!runs.length) return '<tr><td colspan="3" class="muted">no runs</td></tr>';
    return runs.map((r) => "<tr><td class=\"id\">" + esc(r.name) + "</td><td>" + tags(r) + "</td><td>"
      + (r.current || r.cancelled ? "" : '<button data-act="run/current" data-id="' + esc(r.name) + '">make current</button>')
      + (r.cancelled ? "" : '<button class="quiet" data-act="run/cancel" data-id="' + esc(r.name) + '">cancel</button>')
      + "</td></tr>").join("");
  }
  function renderDetail() {
    const e = data.find((x) => x.name === selected);
    if (!e) { $("detail").innerHTML = '<p class="muted">Choose an engagement on the left, or create one.</p>'; return; }
    let h = "<h2>" + esc(e.name) + "</h2><div class=\"muted\">"
      + (e.has_engagement_yaml ? "engagement.yaml present" : "<span class=\"warn\">no engagement.yaml</span>")
      + " · " + (e.has_target ? "target present" : "<span class=\"warn\">no target/</span>") + "</div>";
    h += "<h3>Intakes</h3><table><tr><th>intake</th><th></th><th></th></tr>";
    if (!e.intakes.length) h += '<tr><td colspan="3" class="muted">none — thresholds come from engagement.yaml</td></tr>';
    for (const i of e.intakes) {
      h += "<tr><td class=\"id\">" + esc(i.id) + "</td><td>" + tags(i) + "</td><td>"
        + (i.current || i.cancelled || !i.on_disk ? "" : '<button data-act="intake/current" data-id="' + esc(i.id) + '">make current</button>')
        + (i.cancelled || !i.on_disk ? "" : '<button class="quiet" data-act="intake/cancel" data-id="' + esc(i.id) + '">cancel</button>')
        + "</td></tr>";
      h += '<tr><td colspan="3" style="padding-left:28px"><table>' + runRows(e, i.runs) + "</table></td></tr>";
    }
    h += "</table>";
    if (e.runs_without_intake.length) {
      h += "<h3>Runs without an intake</h3><table>" + runRows(e, e.runs_without_intake) + "</table>";
    }
    h += "<h3>Commands</h3><div class=\"muted\" style=\"margin-bottom:8px\">Click a command to copy it. Nothing runs from this page.</div><div class=\"cmd\">";
    const labels = {intake: "intake", intake_new: "new intake", finish: "finish intake", audit: "audit", review: "review", materiality: "materiality", report: "report", post: "post-delivery page"};
    for (const [k, v] of Object.entries(e.commands)) h += '<div class="k">' + esc(labels[k] || k) + "</div><code>" + esc(v) + "</code>";
    h += "</div>";
    $("detail").innerHTML = h;
    for (const b of document.querySelectorAll("#detail button[data-act]")) {
      b.addEventListener("click", async () => {
        const j = await api("/api/engagements/" + encodeURIComponent(e.name) + "/" + b.dataset.act, {id: b.dataset.id});
        if (j) { data = j; render(); }
      });
    }
    for (const c of document.querySelectorAll("#detail code")) {
      c.addEventListener("click", () => { navigator.clipboard.writeText(c.textContent).then(() => say("copied")); });
    }
  }
  function render() { renderList(); renderDetail(); }

  // #<engagement> in the URL opens that engagement, so a link can point at one.
  (async () => {
    const j = await api("/api/engagements");
    if (j) { data = j; selected = decodeURIComponent(location.hash.slice(1)) || null; render(); }
  })();
})();
