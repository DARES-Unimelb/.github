#!/usr/bin/env node
// Assemble the English report from its parts.
// Usage: node MATLAB/docs/report_sources/en/assemble_en.mjs
//
// Each part file starts with its own H1, which is dropped here and replaced by the
// part heading. Parts 1 and 2 come from translations of the Chinese sources; the
// head and the issues part are written directly in English.

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..', '..', '..');

const read = (name) => {
   const p = join(here, name);
   if (!existsSync(p)) {
      console.error(`missing ${p}`);
      process.exit(1);
   }
   return readFileSync(p, 'utf8').replace(/^\uFEFF/, '').trimEnd();
};

const dropH1 = (text) => {
   const lines = text.split('\n');
   if (lines[0].startsWith('# ')) lines.shift();
   return lines.join('\n').trimStart();
};

const parts = [];
parts.push(read('part_head_en.md'));
parts.push('\n# Part 1. How the control law was designed\n');
parts.push(dropH1(read('part_control_design_en.md')));
parts.push('\n# Part 2. The Simulink bench, block by block\n');
parts.push(dropH1(read('part_simulink_map_en.md')));
parts.push(read('part_issues_en.md'));

const body = parts.join('\n\n').replace(/\n{4,}/g, '\n\n\n') + '\n';
const out = join(root, 'docs', 'design_tutorial_report_en.md');
writeFileSync(out, body, 'utf8');

console.log(`wrote ${out}`);
console.log(`lines ${body.split('\n').length}, bytes ${Buffer.byteLength(body, 'utf8')}`);
