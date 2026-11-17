import { writeFileSync } from 'node:fs';

const UA = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36';
const q = process.argv[2];
const out = process.argv[3] || 'search_out.txt';
const limit = Number(process.argv[4] || 25);

const url = `https://www.mathworks.com/help/search.html?qdoc=${encodeURIComponent(q)}&submitsearch=`;
const res = await fetch(url, {
  headers: { 'User-Agent': UA, 'Accept': 'text/html', 'Accept-Language': 'en-US,en;q=0.9' },
  redirect: 'follow',
});
const html = await res.text();
const links = [...html.matchAll(/href="(\/help\/[^"#?]+)"/g)].map((m) => m[1]);
const uniq = [...new Set(links)].slice(0, limit);
writeFileSync(out, `QUERY: ${q}\nSTATUS: ${res.status}\n\n` + uniq.join('\n') + '\n', 'utf8');
console.log(`QUERY ${q} STATUS ${res.status}`);
console.log(uniq.join('\n'));
