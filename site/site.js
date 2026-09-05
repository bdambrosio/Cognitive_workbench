// Theme: dark unless the visitor chose light; the choice is remembered in this browser.
(function () {
  const root = document.documentElement, key = "theme";
  let saved = null; try { saved = localStorage.getItem(key); } catch (e) {}
  if (saved === "light") root.dataset.theme = "light";
  const btn = document.getElementById("theme");
  if (!btn) return;
  const isDark = () => root.dataset.theme !== "light";
  const label = () => { btn.textContent = isDark() ? "light" : "dark"; };
  btn.addEventListener("click", () => {
    root.dataset.theme = isDark() ? "light" : "dark";
    try { localStorage.setItem(key, root.dataset.theme); } catch (e) {}
    label();
  });
  label();
})();
