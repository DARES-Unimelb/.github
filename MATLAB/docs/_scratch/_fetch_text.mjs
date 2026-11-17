import { writeFileSync } from 'node:fs';

const UA = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36';
const url = process.argv[2];
const out = process.argv[3] || 'page.txt';

const res = await fetch(url, {
  headers: {
    'User-Agent': UA,
    'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8',
    'Accept-Language': 'en-US,en;q=0.9',
  },
  redirect: 'follow',
});
const html = await res.text();

let s = html;
s = s.replace(/<script[\s\S]*?<\/script>/gi, ' ');
s = s.replace(/<style[\s\S]*?<\/style>/gi, ' ');
s = s.replace(/<!--[\s\S]*?-->/g, ' ');
s = s.replace(/<\/(p|div|li|h1|h2|h3|h4|h5|tr|table|section|article|pre)>/gi, '\n');
s = s.replace(/<br\s*\/?>/gi, '\n');
s = s.replace(/<\/(td|th)>/gi, ' | ');
s = s.replace(/<[^>]+>/g, '');
s = s.replace(/&nbsp;/g, ' ');
s = s.replace(/&amp;/g, '&');
s = s.replace(/&lt;/g, '<');
s = s.replace(/&gt;/g, '>');
s = s.replace(/&quot;/g, '"');
s = s.replace(/&#39;/g, "'");
s = s.replace(/[ \t]+/g, ' ');
s = s.replace(/\n\s*\n\s*\n+/g, '\n\n');
writeFileSync(out, `URL: ${url}\nSTATUS: ${res.status}\n\n` + s, 'utf8');
console.log('STATUS', res.status, 'HTML', html.length, 'TEXT', s.length, '->', out);
