// The practice's sorting page: the selection record rendered by md.js, and the
// two lists it proposes, editable, with the button that confirms them.
(function () {
  const qs = location.search || "";
  const $ = (id) => document.getElementById(id);
  $("back").href = "/p/" + qs;
  const lines = (id) => $(id).value.split("\n").map((x) => x.trim()).filter(Boolean);
  function show(j) {
    $("record").innerHTML = window.mdlib.render(j.record);
    for (const t of $("record").querySelectorAll("table")) t.className = "files";
    const lists = j.confirmed || j.proposal;
    $("sources").value = lists.claim_sources.join("\n");
    $("excludes").value = lists.evidence_excludes.join("\n");
    $("confirm").textContent = j.confirmed ? "Confirm again with these lists" : "Confirm the sorting";
  }
  fetch("api" + qs).then((r) => r.ok ? r.json() : r.json().then((e) => Promise.reject(e.detail || r.status)))
    .then(show).catch((e) => { $("record").textContent = String(e); $("confirmBox").hidden = true; });
  $("confirm").addEventListener("click", async () => {
    const r = await fetch("api/confirm" + qs, {method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify({claim_sources: lines("sources"), evidence_excludes: lines("excludes")})});
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { $("msg").textContent = j.detail || ("error " + r.status); return; }
    $("msg").textContent = "confirmed";
    show(j);
  });
})();
