import { writeFileSync } from 'node:fs';

const UA = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36';
const url = process.argv[2];
const out = process.argv[3] || 'probe_out.html';

try {
  const res = await fetch(url, {
    headers: {
      'User-Agent': UA,
      'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8',
      'Accept-Language': 'en-US,en;q=0.9',
    },
    redirect: 'follow',
  });
  const body = await res.text();
  writeFileSync(out, body, 'utf8');
  console.log('STATUS', res.status, 'BYTES', body.length, 'FINAL', res.url);
  console.log('HEAD:', body.slice(0, 300).replace(/\s+/g, ' '));
} catch (e) {
  console.log('ERR', e && e.message);
}
