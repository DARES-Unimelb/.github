#!/usr/bin/env node
// 把分头写的各部分拼成最终报告。
// 用法：node MATLAB/docs/report_sources/assemble.mjs
//
// 每个分册的第一行是它自己的 H1 标题，拼装时丢掉，换成部分的标题。
// 数据表那一份自成一体，整体降一级：它的 H1 变成 H2，H2 变成 H3。

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..', '..');

const read = (name) => {
   const p = join(here, name);
   if (!existsSync(p)) {
      console.error(`缺 ${p}`);
      process.exit(1);
   }
   return readFileSync(p, 'utf8').replace(/^\uFEFF/, '').trimEnd();
};

const dropH1 = (text) => {
   const lines = text.split('\n');
   if (lines[0].startsWith('# ')) lines.shift();
   return lines.join('\n').trimStart();
};

const demote = (text) =>
   text
      .split('\n')
      .map((line) => (/^#{1,5} /.test(line) ? '#' + line : line))
      .join('\n');

const parts = [];
parts.push(read('part_head.md'));
parts.push('\n# 第一部分　控制律是怎么设计出来的\n');
parts.push(dropH1(read('part_control_design.md')));
parts.push('\n# 第二部分　Simulink 台架逐块对照\n');
parts.push(dropH1(read('part_simulink_map.md')));
parts.push('\n# 第三部分　满杆测试与性能\n');
parts.push(read('part_test_method.md'));
parts.push(read('part_analysis.md'));
parts.push(read('part_perf.md'));
parts.push(demote(read('part_data.md')));
parts.push('\n# 第四部分　复核清单与交付说明\n');
parts.push(read('part_check.md'));

const body = parts.join('\n\n').replace(/\n{4,}/g, '\n\n\n') + '\n';
const out = join(root, 'docs', 'design_tutorial_report.md');
writeFileSync(out, body, 'utf8');

const nLines = body.split('\n').length;
console.log(`写出 ${out}`);
console.log(`行数 ${nLines}，字节 ${Buffer.byteLength(body, 'utf8')}`);
