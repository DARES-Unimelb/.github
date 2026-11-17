import { writeFileSync, mkdirSync } from 'node:fs';
import { join } from 'node:path';

const outdir = process.argv[2];
mkdirSync(outdir, { recursive: true });

const UA = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36';
const urls = process.argv.slice(3);
const out = [];
for (const url of urls) {
  const name = url.replace(/^https?:\/\//, '').replace(/[^A-Za-z0-9._-]+/g, '_').slice(0, 110) + '.html';
  try {
    const res = await fetch(url, { headers: { 'User-Agent': UA, 'Accept': '*/*' }, redirect: 'follow' });
    const b = await res.arrayBuffer();
    writeFileSync(join(outdir, name), Buffer.from(b));
    out.push(`${res.status}\t${b.byteLength}\t${name}\t${url}`);
  } catch (e) {
    out.push(`ERR\t0\t-\t${url} :: ${e && e.message}`);
  }
  await new Promise((r) => setTimeout(r, 300));
}
writeFileSync(join(outdir, '_status.txt'), out.join('\n') + '\n', 'utf8');
console.log(out.join('\n'));
