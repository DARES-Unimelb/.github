import { writeFileSync, mkdirSync } from 'node:fs';
import { join } from 'node:path';

const UA = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36';
const outdir = process.argv[2];
mkdirSync(outdir, { recursive: true });
const urls = process.argv.slice(3);

function toText(html) {
  let s = html;
  s = s.replace(/<script[\s\S]*?<\/script>/gi, ' ');
  s = s.replace(/<style[\s\S]*?<\/style>/gi, ' ');
  s = s.replace(/<!--[\s\S]*?-->/g, ' ');
  s = s.replace(/<\/(p|div|li|h1|h2|h3|h4|h5|tr|table|section|article|pre)>/gi, '\n');
  s = s.replace(/<br\s*\/?>/gi, '\n');
  s = s.replace(/<\/(td|th)>/gi, ' | ');
  s = s.replace(/<[^>]+>/g, '');
  s = s.replace(/&nbsp;/g, ' ').replace(/&amp;/g, '&').replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&quot;/g, '"').replace(/&#39;/g, "'");
  s = s.replace(/[ \t]+/g, ' ');
  s = s.replace(/\n\s*\n\s*\n+/g, '\n\n');
  return s;
}

const results = [];
for (const url of urls) {
  const name = url.replace(/^https?:\/\//, '').replace(/[^A-Za-z0-9._-]+/g, '_').slice(0, 120) + '.txt';
  const file = join(outdir, name);
  try {
    const res = await fetch(url, {
      headers: { 'User-Agent': UA, 'Accept': 'text/html,application/xhtml+xml,*/*;q=0.8', 'Accept-Language': 'en-US,en;q=0.9' },
      redirect: 'follow',
    });
    const html = await res.text();
    const text = toText(html);
    writeFileSync(file, `URL: ${url}\nSTATUS: ${res.status}\nFINAL: ${res.url}\n\n` + text, 'utf8');
    results.push(`${res.status}\t${text.length}\t${file}\t${url}`);
  } catch (e) {
    writeFileSync(file, `URL: ${url}\nERROR: ${e && e.message}\n`, 'utf8');
    results.push(`ERR\t0\t${file}\t${url} :: ${e && e.message}`);
  }
  await new Promise((r) => setTimeout(r, 400));
}
writeFileSync(join(outdir, '_index.txt'), results.join('\n') + '\n', 'utf8');
console.log(results.join('\n'));
