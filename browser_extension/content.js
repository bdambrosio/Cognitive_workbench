// Extract page meta description and send to background service worker.
// Runs at document_idle so the DOM is fully parsed.

const description =
  document.querySelector('meta[property="og:description"]')?.content ||
  document.querySelector('meta[name="description"]')?.content ||
  "";

if (description) {
  chrome.runtime.sendMessage({ type: "page_meta", description });
}
