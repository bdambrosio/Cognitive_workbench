// The client's engagement home: the ten stages, what is waiting on whom,
// the letter to accept, and the links that open as stages complete.
(function () {
  const $ = (id) => document.getElementById(id);
  const esc = window.mdlib.esc;
  const qs = location.search || "";
  const LABELS = {
    created: "Engagement opened", letter: "Engagement letter accepted",
    intake: "Intake finished", materials: "Materials in hand",
    enumeration: "Claims enumerated", surface: "Claim surface frozen",
    chain: "Review, check and rating run", release: "Report released",
    closed: "Engagement closed",
  };

  async function api(path, body) {
    const r = await fetch(path + qs, body ? {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)} : {});
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { $("msg").textContent = j.detail || ("error " + r.status); return null; }
    $("msg").textContent = "";
    return j;
  }

  function render(s) {
    $("engagement").textContent = s.name;
    document.title = s.name + " — engagement";
    const n = s.next || {};
    $("next").innerHTML = '<span class="who ' + esc(n.who) + '">' + esc(n.who === "client" ? "your move" : n.who === "practice" ? "with the practice" : "finished") + "</span> " + esc(n.text);
    let h = "";
    for (const st of s.stage_order) {
      const m = s.stages[st];
      const cur = n.stage === st || (n.stage === "report" && st === "release" && !m);
      h += '<li class="' + (m ? "done" : cur ? "current" : "todo") + '"><span class="name">' + esc(LABELS[st] || st) + "</span>"
        + (m ? '<span class="mark">' + esc(m.value) + " · " + esc((m.at || "").replace("T", " ").replace(/-(\d\d)-(\d\d)Z$/, ":$1:$2")) + "</span>" : "") + "</li>";
    }
    $("stagelist").innerHTML = h;
    const v = (st) => (s.stages[st] || {}).value;
    const links = [];
    if (v("letter") === "accepted" || s.role === "practice") links.push(["intake/", v("intake") === "done" ? "The intake conversation (finished)" : "The intake conversation"]);
    if (v("enumeration") === "done" || s.role === "practice") links.push(["surface/", v("surface") === "frozen" ? "The claim surface (frozen)" : "The claim surface: read and comment"]);
    if (s.released || (s.role === "practice" && s.report_exists)) links.push(["report/", "The report: read it and ask questions"]);
    $("links").innerHTML = links.map(([p, t]) => '<a class="btn" href="' + p + qs + '">' + esc(t) + "</a>").join("");
    $("letter").innerHTML = window.mdlib.render(s.letter || "");
    $("accept").hidden = v("letter") === "accepted";
  }

  $("acceptBtn").addEventListener("click", async () => {
    const j = await api("api/letter/accept", {});
    if (j) render(j);
  });

  (async () => { const j = await api("api/status"); if (j) render(j); })();
})();
