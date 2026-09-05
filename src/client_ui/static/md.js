// A small markdown renderer shared by the client pages: paragraphs, headings,
// bullet and numbered lists, fenced code, inline code, bold and italic.
// Everything is escaped first; nothing else is HTML.
(function () {
  function esc(s) {
    return String(s == null ? "" : s).replace(/[&<>"]/g, (c) => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
  }
  function inline(s) {
    return esc(s)
      .replace(/`([^`]+)`/g, "<code>$1</code>")
      .replace(/\*\*([^*]+)\*\*/g, "<strong>$1</strong>")
      .replace(/(^|[^*])\*([^*\n]+)\*(?!\*)/g, "$1<em>$2</em>");
  }
  function render(s) {
    const out = [];
    const lines = String(s || "").replace(/\r/g, "").split("\n");
    let i = 0;
    while (i < lines.length) {
      const l = lines[i];
      if (/^```/.test(l)) {
        const buf = []; i++;
        while (i < lines.length && !/^```/.test(lines[i])) buf.push(lines[i++]);
        i++; out.push("<pre>" + esc(buf.join("\n")) + "</pre>"); continue;
      }
      const h = /^(#{1,3})\s+(.*)$/.exec(l);
      if (h) { out.push("<h" + h[1].length + ">" + inline(h[2]) + "</h" + h[1].length + ">"); i++; continue; }
      if (/^\s*[-*]\s+/.test(l)) {
        const items = [];
        while (i < lines.length && /^\s*[-*]\s+/.test(lines[i])) items.push(lines[i++].replace(/^\s*[-*]\s+/, ""));
        out.push("<ul>" + items.map((t) => "<li>" + inline(t) + "</li>").join("") + "</ul>"); continue;
      }
      if (/^\s*\d+[.)]\s+/.test(l)) {
        const items = [];
        while (i < lines.length && /^\s*\d+[.)]\s+/.test(lines[i])) items.push(lines[i++].replace(/^\s*\d+[.)]\s+/, ""));
        out.push("<ol>" + items.map((t) => "<li>" + inline(t) + "</li>").join("") + "</ol>"); continue;
      }
      if (!l.trim()) { i++; continue; }
      const buf = [];
      while (i < lines.length && lines[i].trim() && !/^(```|#{1,3}\s|\s*[-*]\s|\s*\d+[.)]\s)/.test(lines[i])) buf.push(lines[i++]);
      out.push("<p>" + buf.map(inline).join("<br>") + "</p>");
    }
    return out.join("");
  }
  window.mdlib = {esc, inline, render};
})();
