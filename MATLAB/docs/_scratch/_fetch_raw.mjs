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
  const base = url.split('/').pop().replace(/[^A-Za-z0-9._-]+/g, '_').slice(0, 60);
  const file = join(outdir, base + '.txt');
  try {
    const res = await fetch(url, { headers: { 'User-Agent': UA, 'Accept': '*/*' }, redirect: 'follow' });
    const ct = res.headers.get('content-type') || '';
    let body = Buffer.from(await res.arrayBuffer());
    let text;
    if (/text\/html/i.test(ct)) text = toText(body.toString('utf8'));
    else text = body.toString('utf8');
    writeFileSync(file, `URL: ${url}\nSTATUS: ${res.status}\nCT: ${ct}\n\n` + text, 'utf8');
    results.push(`${res.status}\t${body.length}\t${base}\t${url}`);
  } catch (e) {
    results.push(`ERR\t0\t${base}\t${url} :: ${e && e.message}`);
  }
  await new Promise((r) => setTimeout(r, 300));
}
writeFileSync(join(outdir, '_index.txt'), results.join('\n') + '\n', 'utf8');
console.log(results.join('\n'));
