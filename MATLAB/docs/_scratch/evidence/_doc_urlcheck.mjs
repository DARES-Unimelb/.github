// Link liveness check for the UACS documentation.
// Takes URLs on the command line and prints one TSV row per URL.
// A browser user agent is required: the docs host rejects bare fetch agents with 403.
import { writeFileSync } from 'node:fs';

const UA = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36';
const urls = process.argv.slice(2);
const out = [];
for (const url of urls) {
  try {
    const res = await fetch(url, {
      headers: {
        'User-Agent': UA,
        'Accept': 'text/html,application/xhtml+xml,*/*;q=0.8',
        'Accept-Language': 'en-US,en;q=0.9',
      },
      redirect: 'follow',
    });
    const body = await res.text();
    // Body signal: how many characters survive tag stripping, and whether the
    // page carries a real article rather than a block page.
    const text = body.replace(/<script[\s\S]*?<\/script>/gi, ' ').replace(/<style[\s\S]*?<\/style>/gi, ' ').replace(/<[^>]+>/g, ' ');
    const title = (body.match(/<title[^>]*>([\s\S]*?)<\/title>/i) || [, ''])[1].trim().replace(/\s+/g, ' ');
    out.push([res.status, res.url === url ? '-' : res.url, text.replace(/\s+/g, ' ').trim().length, title.slice(0, 90), url].join('\t'));
  } catch (e) {
    out.push(['ERR', '-', 0, String(e && e.message).slice(0, 90), url].join('\t'));
  }
  await new Promise((r) => setTimeout(r, 300));
}
const report = out.join('\n') + '\n';
writeFileSync(process.argv[1].replace(/\.mjs$/, '.tsv'), report, 'utf8');
console.log(report);
