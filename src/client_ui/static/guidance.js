// The practice's scrub guidance: SCRUB.md rendered by md.js. Practice role only.
(function () {
  const qs = location.search || "";
  document.getElementById("back").href = (document.referrer && document.referrer.indexOf("/p/surface/") >= 0 ? document.referrer : "/p/" + qs);
  fetch("api" + qs).then((r) => r.ok ? r.text() : Promise.reject(r.status)).then((t) => {
    document.getElementById("guidance").innerHTML = window.mdlib.render(t);
  }).catch((e) => { document.getElementById("guidance").textContent = "could not load the guidance (" + e + ")"; });
})();
