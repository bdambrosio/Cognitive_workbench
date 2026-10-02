// The demo page's own controls: the "About this demo" strip remembers whether
// the visitor closed it, and the bar between the chat and the report can be
// dragged to give either more room (double-click restores the default).
// Both choices are kept in this browser only; a blocked localStorage just
// means the page opens with the defaults.
(function () {
  "use strict";
  function load(key) { try { return localStorage.getItem(key); } catch (e) { return null; } }
  function save(key, value) { try { localStorage.setItem(key, value); } catch (e) { /* not kept */ } }

  const about = document.getElementById("about");
  if (about) {
    if (load("demo.about") === "closed") about.open = false;
    about.addEventListener("toggle", () => save("demo.about", about.open ? "open" : "closed"));
  }

  const main = document.querySelector("main");
  const bar = document.getElementById("splitter");
  if (!main || !bar) return;
  const MIN = 22, MAX = 60, DEFAULT = 34;            // the chat pane's share, in percent
  function setShare(pct) {
    const v = Math.min(MAX, Math.max(MIN, pct));
    main.style.setProperty("--chat-share", v + "%");
    return v;
  }
  const kept = parseFloat(load("demo.chatShare"));
  setShare(isFinite(kept) ? kept : DEFAULT);

  let dragging = false;
  bar.addEventListener("pointerdown", (ev) => {
    dragging = true; bar.setPointerCapture(ev.pointerId); document.body.classList.add("resizing");
  });
  bar.addEventListener("pointermove", (ev) => {
    if (!dragging) return;
    const r = main.getBoundingClientRect();
    setShare(((ev.clientX - r.left) / r.width) * 100);
  });
  function stop(ev) {
    if (!dragging) return;
    dragging = false; document.body.classList.remove("resizing");
    try { bar.releasePointerCapture(ev.pointerId); } catch (e) { /* already released */ }
    save("demo.chatShare", parseFloat(main.style.getPropertyValue("--chat-share")));
  }
  bar.addEventListener("pointerup", stop);
  bar.addEventListener("pointercancel", stop);
  bar.addEventListener("dblclick", () => save("demo.chatShare", setShare(DEFAULT)));
  bar.addEventListener("keydown", (ev) => {
    const cur = parseFloat(main.style.getPropertyValue("--chat-share")) || DEFAULT;
    if (ev.key === "ArrowLeft" || ev.key === "ArrowRight") {
      ev.preventDefault();
      save("demo.chatShare", setShare(cur + (ev.key === "ArrowRight" ? 2 : -2)));
    }
  });
})();
