The target: Cognitive_workbench, the repository holding the Tuuyi claims-review pipeline (workflowsv2/), the chat agent runtime it runs on (src/chat/), the client site (src/client_ui/), the public demo (src/demo/) and the measurement harness (measure/), cloned locally at commit 3b57e91d on 2026-09-17.

Claim sources supplied: the text of the practice's public website, https://tuuyi.com, one file per page under claim_sources/site/, extracted from site/*.html at the same commit. The introduction page carries a nine-minute video; its narration is printed on the page as a transcript, and that transcript is a claim source of its own. Each file opens with the page's title and its meta description, which search engines show; the home page's file also carries the one banner sentence shown in the header of every page. Left out: the not-found page, which no link on the site and no sitemap entry leads to, so no visitor reaches it except by a wrong address; text shown only on screen in the video, which has no written form to quote; demo.tuuyi.com, a separate host serving a delivered report about another project; the video's caption file and the link-preview image, whose words are all in the included text.

What the buyer knows or suspects: nothing stated; a self-review engagement. Many statements on the site are about how the practice works (price, timing, who signs, where paid work runs) and cannot be settled from a repository.

Scope agreed: the claims on the listed pages against the repository.

Deliverable: the standard report.
