// Contact form and opt-out receiver for tuuyi.com. Runs on Cloudflare
// Workers at tuuyi.com/api/contact and tuuyi.com/api/optout.
//
// /api/contact takes the POST from /contact, checks the Turnstile token, and
// mails the submission to the practice through the Worker email binding.
// Responds with a redirect back to /contact so the page can show the outcome.
//
// /api/optout takes a request not to be emailed again, from the form on
// /optout or from a mail program's one-click unsubscribe (RFC 8058: a POST
// whose body is "List-Unsubscribe=One-Click", the address in the URL's `e`).
// It has no Turnstile check, because a one-click POST cannot carry one. It
// mails the address to the practice, which records it by hand. The form gets
// a redirect back to /optout; a one-click POST gets 200.
//
// Nothing is stored.
//
// Bindings (set in deploy.sh): SEND (send_email, destination = the practice's
// inbox), TURNSTILE_SECRET (secret text).

import { EmailMessage } from "cloudflare:email";

const FROM = "contact@tuuyi.com";
const TO = "info@tuuyi.com";
const PAGE = "https://tuuyi.com/contact";
const OPTOUT_PAGE = "https://tuuyi.com/optout";
const LIMITS = { name: 200, email: 254, company: 300, link: 500, message: 5000 };
const REASONS = { beta: "Beta claims review", seller: "Seller claims check", contact: "Contact" };

export default {
  async fetch(request, env) {
    if (new URL(request.url).pathname.startsWith("/api/optout")) {
      return optout(request, env);
    }
    if (request.method !== "POST") {
      return Response.redirect(PAGE, 303);
    }
    if (Number(request.headers.get("content-length") || 0) > 20000) {
      return back("error=fields");
    }
    let form;
    try {
      form = await request.formData();
    } catch (e) {
      return back("error=fields");
    }
    const f = (k) => (form.get(k) || "").toString().trim();
    const fields = {
      name: f("name"), email: f("email"), company: f("company"),
      link: f("link"), message: f("message"),
    };
    const reason = REASONS[f("reason")] || REASONS.contact;
    if (!fields.name || !fields.email || !fields.message ||
        !/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(fields.email) ||
        Object.entries(LIMITS).some(([k, n]) => fields[k].length > n)) {
      return back("error=fields");
    }
    const ok = await turnstileOk(f("cf-turnstile-response"),
                                 request.headers.get("CF-Connecting-IP"),
                                 env.TURNSTILE_SECRET);
    if (!ok) {
      return back("error=check");
    }
    try {
      await env.SEND.send(new EmailMessage(FROM, TO, mime(reason, fields)));
    } catch (e) {
      console.log("send failed: " + (e && e.message));
      return back("error=send");
    }
    return back("sent=1");
  },
};

async function optout(request, env) {
  if (request.method !== "POST") {
    return Response.redirect(OPTOUT_PAGE, 303);
  }
  let form;
  try {
    form = await request.formData();
  } catch (e) {
    form = new FormData();
  }
  const oneClick = (form.get("List-Unsubscribe") || "").toString() === "One-Click";
  const address = ((form.get("e") || new URL(request.url).searchParams.get("e") || "") + "").trim();
  const done = (query) => oneClick ? new Response(query === "done=1" ? "ok" : "bad request",
                                                  { status: query === "done=1" ? 200 : 400 })
                                   : Response.redirect(OPTOUT_PAGE + "?" + query, 303);
  if (!address || address.length > LIMITS.email || !/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(address)) {
    return done("error=fields");
  }
  const how = oneClick ? "the unsubscribe control of their mail program" : "the form at tuuyi.com/optout";
  try {
    await env.SEND.send(new EmailMessage(FROM, TO, message(
      `[tuuyi.com] opt-out: ${address}`,
      `Opt-out request\nAddress: ${address}\nSent through: ${how}\n\n` +
      `Record it on the outreach page: press "Opted out" on this person's card.\n`, null)));
  } catch (e) {
    console.log("send failed: " + (e && e.message));
    return done("error=send");
  }
  return done("done=1");
}

function back(query) {
  return Response.redirect(PAGE + "?" + query, 303);
}

async function turnstileOk(token, ip, secret) {
  if (!token) return false;
  const body = new URLSearchParams({ secret, response: token });
  if (ip) body.set("remoteip", ip);
  const r = await fetch("https://challenges.cloudflare.com/turnstile/v0/siteverify",
                        { method: "POST", body });
  const j = await r.json().catch(() => ({}));
  return j.success === true;
}

// A plain-text RFC 5322 message. The body is base64 so any character the
// visitor typed survives; the subject is RFC 2047 encoded for the same reason.
function mime(reason, x) {
  const text =
    `Reason: ${reason}\n` +
    `Name: ${x.name}\n` +
    `Email: ${x.email}\n` +
    `Company: ${x.company || "-"}\n` +
    `Link: ${x.link || "-"}\n\n` +
    `${x.message}\n`;
  return message(`[tuuyi.com] ${reason} from ${x.name}`, text, x.email);
}

function message(subject, text, replyTo) {
  return [
    `From: Tuuyi contact form <${FROM}>`,
    `To: <${TO}>`,
    ...(replyTo ? [`Reply-To: <${replyTo.replace(/[\r\n<>]/g, "")}>`] : []),
    `Subject: =?utf-8?B?${b64(subject)}?=`,
    `Date: ${new Date().toUTCString()}`,
    `Message-ID: <${crypto.randomUUID()}@tuuyi.com>`,
    `MIME-Version: 1.0`,
    `Content-Type: text/plain; charset=utf-8`,
    `Content-Transfer-Encoding: base64`,
    ``,
    b64(text).replace(/(.{76})/g, "$1\r\n"),
    ``,
  ].join("\r\n");
}

function b64(s) {
  const bytes = new TextEncoder().encode(s);
  let bin = "";
  for (const b of bytes) bin += String.fromCharCode(b);
  return btoa(bin);
}
