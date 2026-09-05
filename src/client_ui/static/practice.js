// The practice page: engagements, their intakes and runs, the current choices, the commands.
(function () {
  const token = new URLSearchParams(location.search).get("token") || "";
  const $ = (id) => document.getElementById(id);
  let data = [], selected = null;
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"]/g, (c) => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
  // Addresses are relative to the page: /api/... on the practice page,
  // /p/api/... on the site. The token rides along where there is one.
  const q = (path) => path + (token ? (path.includes("?") ? "&" : "?") + "token=" + encodeURIComponent(token) : "");

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
      + '<input id="newEmails" placeholder="client emails, comma-separated">'
      + '<button id="newBtn">Create</button></div>';
    $("list").innerHTML = h;
    for (const el of document.querySelectorAll(".eng")) el.addEventListener("click", () => { selected = el.dataset.name; render(); });
    $("newBtn").addEventListener("click", async () => {
      const name = $("newName").value.trim(), clone = $("newClone").value.trim();
      const client_emails = $("newEmails").value.split(",").map((x) => x.trim()).filter(Boolean);
      if (!name) return;
      const j = await api("api/engagements", {name, clone: clone || null, client_emails});
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
  const STAGE_LABELS = {created: "opened", letter: "letter accepted", intake: "intake finished", materials: "materials ready",
    enumeration: "claims enumerated", surface: "surface frozen", chain: "review run", release: "report released", closed: "closed"};
  function stagePanel(e) {
    const v = (st) => (e.stages[st] || {}).value;
    const job = e.job;
    const n = e.next || {};
    let h = "<h3>Stages</h3><div class=\"next\"><span class=\"muted\">" + esc(n.who === "client" ? "waiting on the client" : n.who === "practice" ? "the practice's move" : "finished") + "</span> · " + esc(n.text) + "</div>";
    h += "<table class=\"stages\">";
    for (const st of ["created", "letter", "intake", "materials", "enumeration", "surface", "chain", "release", "closed"]) {
      const m = e.stages[st];
      h += "<tr><td class=\"id\">" + esc(STAGE_LABELS[st]) + "</td><td>" + (m ? esc(m.value) + ' <span class="muted">' + esc((m.at || "").replace("T", " ")) + " " + esc(m.by || "") + "</span>" : '<span class="muted">—</span>') + "</td></tr>";
    }
    h += "</table>";
    if (job) h += '<div class="job">job <b>' + esc(job.kind) + "</b> running since " + esc(job.started) + ' · <a target="_blank" href="' + q("api/engagements/" + encodeURIComponent(e.name) + "/jobs/" + encodeURIComponent(job.id) + "/log") + '">log</a></div>';
    const last = (e.jobs || []).slice(-1)[0];
    if (!job && last) h += '<div class="job muted">last job: ' + esc(last.kind) + " " + esc(last.state) + (last.error ? " — " + esc(last.error) : "") + ' · <a target="_blank" href="' + q("api/engagements/" + encodeURIComponent(e.name) + "/jobs/" + encodeURIComponent(last.id) + "/log") + '">log</a></div>';
    h += '<div class="buttons">';
    const btn = (act, label, on) => on ? '<button data-site="' + act + '">' + esc(label) + "</button>" : "";
    h += btn("stage:materials:ready", "Materials ready", v("intake") === "done" && v("materials") !== "ready");
    h += btn("job:enumerate", "Run enumeration", v("materials") === "ready" && !job && v("surface") !== "frozen");
    h += btn("job:chain", "Run the review", v("surface") === "frozen" && !job && v("chain") !== "done");
    h += btn("stage:release:released", "Release to the client", v("chain") === "done" && e.report_exists && v("release") !== "released");
    h += btn("stage:closed:closed", "Close the engagement", v("release") === "released" && v("closed") !== "closed");
    h += "</div>";
    h += '<div class="pages">';
    h += '<a href="' + q("/e/" + encodeURIComponent(e.name) + "/") + '">client home</a>';
    h += ' · <a href="' + q("/e/" + encodeURIComponent(e.name) + "/intake/") + '">intake</a>';
    h += ' · <a href="' + q("/p/surface/" + encodeURIComponent(e.name) + "/") + '">surface editor</a>';
    if (e.report_exists) h += ' · <a href="' + q("/e/" + encodeURIComponent(e.name) + "/report/") + '">report</a>';
    h += "</div>";
    const srcs = (e.surfaces || []).map((s) => esc(s.source) + " (" + s.claims + (s.frozen ? ", frozen" : "") + ")").join(", ");
    h += '<div class="muted">claim sources: ' + (srcs || "none in engagement.yaml") + "</div>";
    const st = e.settings || {};
    h += '<h3>Settings</h3><div class="settings">'
      + '<label>claim sources, one per line, by path from the target root<textarea id="setSources" rows="3">' + esc((st.claim_sources || []).join("\n")) + "</textarea></label>"
      + '<label>client emails, comma-separated (new ones are added to the Access policy and mailed the link)<input id="setEmails" value="' + esc((st.client_emails || []).join(", ")) + '"></label>'
      + '<label>target (path; "target" is the engagement\'s own clone)<input id="setTarget" value="' + esc(st.target || "") + '"></label>'
      + '<label>retention<input id="setRetention" value="' + esc(st.retention || "") + '"></label>'
      + '<label>engagement letter' + (st.letter_is_template ? ' <span class="muted">(empty: the template is shown)</span>' : "") + '<textarea id="setLetter" rows="6" placeholder="Leave empty to show the practice\'s template letter.">' + esc(st.letter || "") + "</textarea></label>"
      + '<button id="setSave">Save settings</button></div>';
    return h;
  }
  function renderDetail() {
    const e = data.find((x) => x.name === selected);
    if (!e) { $("detail").innerHTML = '<p class="muted">Choose an engagement on the left, or create one.</p>'; return; }
    let h = "<h2>" + esc(e.name) + "</h2><div class=\"muted\">"
      + (e.has_engagement_yaml ? "engagement.yaml present" : "<span class=\"warn\">no engagement.yaml</span>")
      + " · " + (e.has_target ? "target present" : "<span class=\"warn\">no target/</span>") + "</div>";
    if (e.site) h += stagePanel(e);
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
    h += "<h3>Commands</h3><div class=\"muted\" style=\"margin-bottom:8px\">Click a command to copy it. "
      + (e.site ? "The buttons above start the jobs; these are the same steps for a terminal." : "Nothing runs from this page.") + "</div><div class=\"cmd\">";
    const labels = {intake: "intake", intake_new: "new intake", finish: "finish intake", audit: "audit", review: "review", materiality: "materiality", report: "report", post: "post-delivery page"};
    for (const [k, v] of Object.entries(e.commands)) h += '<div class="k">' + esc(labels[k] || k) + "</div><code>" + esc(v) + "</code>";
    h += "</div>";
    $("detail").innerHTML = h;
    for (const b of document.querySelectorAll("#detail button[data-act]")) {
      b.addEventListener("click", async () => {
        const j = await api("api/engagements/" + encodeURIComponent(e.name) + "/" + b.dataset.act, {id: b.dataset.id});
        if (j) { data = j; render(); }
      });
    }
    if ($("setSave")) $("setSave").addEventListener("click", async () => {
      const body = {
        claim_sources: $("setSources").value.split("\n").map((x) => x.trim()).filter(Boolean),
        client_emails: $("setEmails").value.split(",").map((x) => x.trim()).filter(Boolean),
        target: $("setTarget").value.trim(), retention: $("setRetention").value.trim(),
        letter: $("setLetter").value,
      };
      const j = await api("api/engagements/" + encodeURIComponent(e.name) + "/settings", body);
      if (j) { data = j.engagements; render(); say(j.policy ? ("saved · Access policy: " + j.policy.reason + (j.policy.added.length ? " " + j.policy.added.join(", ") : "")) : "saved"); }
    });
    for (const b of document.querySelectorAll("#detail button[data-site]")) {
      b.addEventListener("click", async () => {
        const [what, a, val] = b.dataset.site.split(":");
        const path = "api/engagements/" + encodeURIComponent(e.name) + (what === "job" ? "/jobs/" + a : "/stage");
        if (what === "job" && !confirm("Start the " + a + " job for " + e.name + "?")) return;
        const j = await api(path, what === "job" ? {} : {stage: a, value: val});
        if (j) { data = j.engagements || j; render(); }
      });
    }
    for (const c of document.querySelectorAll("#detail code")) {
      c.addEventListener("click", () => { navigator.clipboard.writeText(c.textContent).then(() => say("copied")); });
    }
  }
  function render() { renderList(); renderDetail(); }

  // #<engagement> in the URL opens that engagement, so a link can point at one.
  (async () => {
    const j = await api("api/engagements");
    if (j) { data = j; selected = decodeURIComponent(location.hash.slice(1)) || null; render(); }
  })();
})();
