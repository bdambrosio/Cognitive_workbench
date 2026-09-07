// The materials page: the files under the engagement's target, for the
// seller and the practice. Navigate, upload files or a folder, make a
// folder, download, delete with confirmation; the practice marks claim
// sources and exclusions; the seller says when the materials are complete.
(function () {
  const $ = (id) => document.getElementById(id);
  const esc = window.mdlib.esc;
  const qs = location.search || "";
  const base = "api";
  let cur = null;                       // the last listing

  function say(t, warn) { const m = $("msg"); m.textContent = t; m.className = warn ? "warn" : ""; }

  // The identity query (?as=, with no access check) rides on every call.
  const q = (path) => path + (qs ? (path.includes("?") ? "&" : "?") + qs.slice(1) : "");

  async function api(path, body) {
    const r = await fetch(q(path), body ? {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)} : {});
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { say(j.detail || ("error " + r.status), true); return null; }
    say("");
    return j;
  }

  function here() { return decodeURIComponent((location.hash || "#").slice(1)); }
  function go(path) { location.hash = "#" + encodeURIComponent(path); }

  function size(n) {
    if (n < 1024) return n + " B";
    if (n < 1024 * 1024) return (n / 1024).toFixed(0) + " KB";
    return (n / (1024 * 1024)).toFixed(1) + " MB";
  }
  function when(t) { return new Date(t * 1000).toISOString().slice(0, 16).replace("T", " "); }

  function render(s) {
    cur = s;
    const practice = s.roles.includes("practice");
    $("engagement").textContent = s.name;
    document.title = s.name + " — materials";
    $("back").href = "../" + qs;
    const n = s.next || {};
    $("next").innerHTML = '<span class="who ' + esc(n.who) + '">' + esc(n.who === "seller" ? "with the seller" : n.who === "client" ? "with the buyer" : n.who === "practice" ? "with the practice" : "finished") + "</span> " + esc(n.text);
    $("intro").textContent = practice
      ? "Everything under the engagement's target, as the review will read it. The seller sees and maintains this page too; the buyer never does."
      : "Upload the repository and the documents to be examined and arrange them as you like. The practice sees this page; the buyer does not. The report names files by path and quotes the lines each finding rests on.";

    // breadcrumbs
    const parts = s.path ? s.path.split("/") : [];
    let c = '<a href="#" data-go="">' + esc(s.name) + "</a>";
    let acc = "";
    for (const p of parts) { acc = acc ? acc + "/" + p : p; c += ' / <a href="#" data-go="' + esc(acc) + '">' + esc(p) + "</a>"; }
    $("crumbs").innerHTML = c;
    for (const a of document.querySelectorAll("#crumbs a")) a.addEventListener("click", (ev) => { ev.preventDefault(); go(a.dataset.go); });

    // tools
    $("locked").hidden = s.writable;
    $("locked").textContent = s.writable ? "" : "Read only: " + s.why_not_writable + ".";
    $("tools").innerHTML = s.writable
      ? '<label class="btn">Upload files<input type="file" id="upFiles" multiple hidden></label>'
        + '<label class="btn">Upload a folder<input type="file" id="upDir" webkitdirectory multiple hidden></label>'
        + '<button id="mkdir">New folder</button>'
      : "";
    if (s.writable) {
      $("upFiles").addEventListener("change", (ev) => upload(ev.target.files));
      $("upDir").addEventListener("change", (ev) => upload(ev.target.files));
      $("mkdir").addEventListener("click", async () => {
        const name = prompt("Name of the new folder:");
        if (!name || !name.trim()) return;
        const j = await api(base + "/mkdir", {path: (s.path ? s.path + "/" : "") + name.trim()});
        if (j) load();
      });
    }

    // the listing
    let h = "<tr><th>name</th><th>size</th><th>modified</th>" + (practice ? "<th>claim source</th><th>excluded</th>" : "") + "<th></th></tr>";
    if (s.path) h += '<tr><td><a href="#" data-go="' + esc(parts.slice(0, -1).join("/")) + '">..</a></td><td></td><td></td>' + (practice ? "<td></td><td></td>" : "") + "<td></td></tr>";
    if (!s.entries.length) h += '<tr><td colspan="6" class="muted">' + (s.root_exists ? "empty" : "no materials yet") + "</td></tr>";
    for (const e of s.entries) {
      const name = e.dir ? '<a href="#" data-go="' + esc(e.path) + '">' + esc(e.name) + "/</a>"
        : '<a href="' + esc(q(base + "/file?path=" + encodeURIComponent(e.path))) + '">' + esc(e.name) + "</a>";
      h += '<tr class="' + (e.excluded || e.under_excluded ? "excluded" : "") + '"><td>' + name + "</td><td class=\"num\">" + (e.dir ? "" : size(e.size)) + "</td><td class=\"mono\">" + when(e.mtime) + "</td>";
      if (practice) {
        h += '<td><input type="checkbox" data-mark="claim_source" data-path="' + esc(e.path) + '"' + (e.claim_source ? " checked" : "") + (e.dir ? ' title="marks the files directly in this folder"' : "") + "></td>";
        h += '<td><input type="checkbox" data-mark="excluded" data-path="' + esc(e.path) + '"' + (e.excluded ? " checked" : "") + (e.under_excluded ? " disabled title=\"inside an excluded folder\"" : "") + "></td>";
      }
      h += "<td>" + (s.writable ? '<button class="quiet" data-del="' + esc(e.path) + '">delete</button>' : "") + "</td></tr>";
    }
    $("files").innerHTML = h;
    for (const a of document.querySelectorAll("#files a[data-go]")) a.addEventListener("click", (ev) => { ev.preventDefault(); go(a.dataset.go); });
    for (const b of document.querySelectorAll("#files button[data-del]")) b.addEventListener("click", async () => {
      const p = b.dataset.del;
      const isDir = s.entries.find((e) => e.path === p)?.dir;
      if (!confirm("Delete " + p + (isDir ? " and everything in it" : "") + "? This cannot be undone.")) return;
      const j = await api(base + "/delete", {path: p});
      if (j) load();
    });
    for (const cb of document.querySelectorAll("#files input[data-mark]")) cb.addEventListener("change", async () => {
      const body = {path: cb.dataset.path};
      body[cb.dataset.mark] = cb.checked;
      const j = await api(base + "/mark", body);
      if (j) load(); else cb.checked = !cb.checked;
    });

    // the seller's word, and the summary of the marks
    let a = "";
    if (s.writable && s.materials !== "supplied" && s.materials !== "ready") a += '<button id="supplied">The materials are complete</button>';
    if (s.materials === "supplied") a += '<span class="muted">Marked complete; the practice is looking them over.</span>';
    if (s.materials === "ready") a += '<span class="muted">The practice has marked the materials ready.</span>';
    a += '<span class="muted">' + s.count + " file" + (s.count === 1 ? "" : "s") + " in all</span>";
    $("actions").innerHTML = a;
    if ($("supplied")) $("supplied").addEventListener("click", async () => {
      if (!confirm("Tell the practice the materials are complete?")) return;
      const j = await api(base + "/supplied", {});
      if (j) load();
    });
    $("marks").hidden = !practice;
    if (practice) {
      $("markLists").innerHTML = "claim sources: " + (s.claim_sources.length ? esc(s.claim_sources.join(", ")) : '<span class="muted">none</span>')
        + "<br>excluded: " + (s.evidence_excludes.length ? esc(s.evidence_excludes.join(", ")) : '<span class="muted">none</span>')
        + (s.excludes_explicit ? "" : ' <span class="muted">(defaulting to the claim sources)</span>');
    }
  }

  async function upload(files) {
    const list = Array.from(files || []);
    if (!list.length) return;
    let n = 0;
    for (const f of list) {
      n += 1;
      say("uploading " + n + " of " + list.length + ": " + (f.webkitRelativePath || f.name));
      const body = new FormData();
      body.append("into", cur ? cur.path : "");
      body.append("path", f.webkitRelativePath || f.name);
      body.append("file", f, f.name);
      const r = await fetch(q(base + "/upload"), {method: "POST", body});
      if (!r.ok) {
        const j = await r.json().catch(() => ({}));
        await load();
        say("upload of " + (f.webkitRelativePath || f.name) + " failed: " + (j.detail || r.status), true);
        return;
      }
    }
    await load();
    say("uploaded " + list.length + " file" + (list.length === 1 ? "" : "s"));
  }

  async function load() {
    const j = await api(base + "?path=" + encodeURIComponent(here()));
    if (j) render(j);
  }

  window.addEventListener("hashchange", load);
  load();
})();
