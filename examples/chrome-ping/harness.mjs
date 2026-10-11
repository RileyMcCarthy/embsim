// A harness for the run, as a test runner drives a page the board's clock
// meters: Playwright connects over DevTools to the Chrome the run launched,
// finds the page the run opened, waits in the page's own time, and reads
// it.
//
//   embsim run project.toml --for 30s        # prints "… DevTools at http://127.0.0.1:PORT …"
//   node harness.mjs http://127.0.0.1:PORT   # in another terminal, once that line is out
//
// Needs Playwright (`npm install playwright`); it drives the Chrome embsim
// launched, so no browser download is needed. A harness that makes pages of
// its own makes a fresh context for each scenario (`browser.newContext()`,
// then `newPage()` and `goto()`), which gives each page a window of its own.
import { chromium } from 'playwright';

const endpoint = process.argv[2];
if (!endpoint) {
  console.error('usage: node harness.mjs http://127.0.0.1:PORT (the DevTools endpoint the run printed)');
  process.exit(2);
}

// Wait until the page's own clock has advanced `ms`: the board's time, not
// the host's. Rendering barely runs under virtual time, so poll on a timer,
// never on animation frames.
async function waitPageTime(page, ms) {
  const start = await page.evaluate(() => performance.now());
  await page.waitForFunction((until) => performance.now() >= until, start + ms, {
    polling: 50,
    timeout: 0,
  });
}

const browser = await chromium.connectOverCDP(endpoint);
const page = browser
  .contexts()
  .flatMap((context) => context.pages())
  .find((p) => p.url().endsWith('/ping.html'));
if (!page) {
  console.error('no ping.html page in that browser');
  process.exit(1);
}
const before = await page.textContent('#state');
await waitPageTime(page, 500);
const after = await page.textContent('#state');
console.log(`the page said "${before}", and 500 ms of its time later "${after}"`);
await browser.close();
