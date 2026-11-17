const UA = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36';
const urls = process.argv.slice(2);
for (const u of urls) {
  const t0 = Date.now();
  try {
    const res = await fetch(u, { headers: { 'User-Agent': UA }, redirect: 'follow', method: 'GET' });
    const body = await res.arrayBuffer();
    console.log(`${res.status}\t${body.byteLength}\t${Date.now() - t0}ms\t${u}`);
  } catch (e) {
    console.log(`ERR\t0\t-\t${u} :: ${e && e.message}`);
  }
  await new Promise((r) => setTimeout(r, 200));
}
