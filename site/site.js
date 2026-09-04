// Theme: follows the system unless the visitor chose; the choice is remembered in this browser.
(function () {
  const root = document.documentElement, key = "theme";
  let saved = null; try { saved = localStorage.getItem(key); } catch (e) {}
  if (saved) root.dataset.theme = saved;
  const btn = document.getElementById("theme");
  if (!btn) return;
  const isDark = () => root.dataset.theme === "dark" || (!root.dataset.theme && matchMedia("(prefers-color-scheme: dark)").matches);
  const label = () => { btn.textContent = isDark() ? "light" : "dark"; };
  btn.addEventListener("click", () => {
    root.dataset.theme = isDark() ? "light" : "dark";
    try { localStorage.setItem(key, root.dataset.theme); } catch (e) {}
    label();
  });
  label();
})();
