// The engagement home: the stages, what is waiting on whom, the letter to
// accept, and the links that open as stages complete. The client (the
// buyer) sees the letter, intake, surface and report; the seller sees the
// materials page; an address with both roles sees both.
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
    const roles = s.roles || [s.role];
    const mine = (n.who === "client" && roles.includes("client")) || (n.who === "seller" && roles.includes("seller"));
    $("next").innerHTML = '<span class="who ' + esc(n.who) + '">' + esc(mine ? "your move" : n.who === "client" ? "with the buyer" : n.who === "seller" ? "with the seller" : n.who === "practice" ? "with the practice" : "finished") + "</span> " + esc(n.text);
    let h = "";
    for (const st of s.stage_order) {
      const m = s.stages[st];
      const cur = n.stage === st || (n.stage === "report" && st === "release" && !m);
      h += '<li class="' + (m ? "done" : cur ? "current" : "todo") + '"><span class="name">' + esc(LABELS[st] || st) + "</span>"
        + (m ? '<span class="mark">' + esc(m.value) + " · " + esc((m.at || "").replace("T", " ").replace(/-(\d\d)-(\d\d)Z$/, ":$1:$2")) + "</span>" : "") + "</li>";
    }
    $("stagelist").innerHTML = h;
    const v = (st) => (s.stages[st] || {}).value;
    const client = roles.includes("client") || roles.includes("practice");
    const links = [];
    if (client && (v("letter") === "accepted" || s.role === "practice")) links.push(["intake/", v("intake") === "done" ? "The intake conversation (finished)" : "The intake conversation"]);
    if (roles.includes("seller") || roles.includes("practice")) links.push(["materials/", v("materials") === "ready" ? "The materials (marked ready)" : "The materials: upload and arrange them"]);
    if (client && (v("enumeration") === "done" || s.role === "practice")) links.push(["surface/", v("surface") === "frozen" ? "The claim surface (frozen)" : "The claim surface: read and comment"]);
    if (client && (s.released || (s.role === "practice" && s.report_exists))) links.push(["report/", "The report: read it and ask questions"]);
    $("links").innerHTML = links.map(([p, t]) => '<a class="btn" href="' + p + qs + '">' + esc(t) + "</a>").join("");
    // the seller alone has no letter to read: one column, no pane
    $("letterPane").style.display = client ? "" : "none";
    document.querySelector("main.site").classList.toggle("one", !client);
    $("stages").style.borderRight = client ? "" : "none";
    $("letter").innerHTML = client ? window.mdlib.render(s.letter || "") : "";
    $("accept").hidden = !client || v("letter") === "accepted";
  }

  $("acceptBtn").addEventListener("click", async () => {
    const j = await api("api/letter/accept", {});
    if (j) render(j);
  });

  (async () => { const j = await api("api/status"); if (j) render(j); })();
})();
