const LISTENER_URL = "http://localhost:5004/page-visit";
const RETRY_INTERVAL_MS = 10000;
const MAX_BUFFER_SIZE = 500;

let buffer = [];
let retrying = false;

chrome.tabs.onUpdated.addListener((tabId, changeInfo, tab) => {
  if (changeInfo.status !== "complete" || !tab.url) return;

  const entry = {
    url: tab.url,
    title: tab.title || "",
    timestamp: new Date().toISOString(),
    tabId: tabId,
  };

  sendOrBuffer(entry);
});

async function sendOrBuffer(entry) {
  try {
    const response = await fetch(LISTENER_URL, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(entry),
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);

    // If send succeeded and there are buffered entries, flush them
    if (buffer.length > 0) flushBuffer();
  } catch (e) {
    buffer.push(entry);
    if (buffer.length > MAX_BUFFER_SIZE) buffer.shift();
    scheduleRetry();
  }
}

async function flushBuffer() {
  const entries = [...buffer];
  buffer = [];
  try {
    const response = await fetch(LISTENER_URL, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(entries),
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
  } catch (e) {
    // Put them back at the front
    buffer = entries.concat(buffer);
    if (buffer.length > MAX_BUFFER_SIZE) buffer.splice(0, buffer.length - MAX_BUFFER_SIZE);
    scheduleRetry();
  }
}

function scheduleRetry() {
  if (retrying) return;
  retrying = true;
  setTimeout(async () => {
    retrying = false;
    if (buffer.length > 0) flushBuffer();
  }, RETRY_INTERVAL_MS);
}
