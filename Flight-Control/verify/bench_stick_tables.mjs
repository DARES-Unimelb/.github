#!/usr/bin/env node
// BENCH_STICK_TABLES Turn the full stick metric table into a markdown report table.
//
//   node verify/bench_stick_tables.mjs
//
// Input   results/bench_stick_metrics.csv   written by verify/bench_stick_report.m
// Output  docs/report_sources/part_data.md
//
// Columns are read by name, never by index.

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const root = dirname(here);
const csvPath = join(root, 'results', 'bench_stick_metrics.csv');
const outPath = join(root, 'docs', 'report_sources', 'part_data.md');

if (!existsSync(csvPath)) {
   console.error(`缺输入文件 ${csvPath}`);
   process.exit(1);
}

const text = readFileSync(csvPath, 'utf8').replace(/^\uFEFF/, '');
const lines = text.split(/\r?\n/).filter((s) => s.trim() !== '');
const head = lines[0].split(',');
const rows = lines.slice(1).map((line) => {
   const parts = line.split(',');
   const o = {};
   head.forEach((h, i) => {
      const v = parts[i];
      o[h] = Number.isNaN(Number(v)) ? v : Number(v);
   });
   return o;
});

const n = (v, d = 2) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '');
const sgn = (v, d = 2) => (typeof v === 'number' && Number.isFinite(v) ? (v >= 0 ? '+' : '') + v.toFixed(d) : '');
const pct = (v) => (typeof v === 'number' && Number.isFinite(v) ? (v * 100).toFixed(0) + '%' : '');
const ctrlName = (c) => (c === 0 ? 'PID' : 'LQR');

const family = (id) => {
   if (id === 1) return '基准';
   if (id <= 7) return '单轴满杆';
   if (id <= 19) return '两轴组合';
   return '三轴组合';
};
const families = ['基准', '单轴满杆', '两轴组合', '三轴组合'];

const out = [];
out.push('# 满杆矩阵数据表（自动生成）');
out.push('');
out.push(`共 ${rows.length} 段，每段仿真 ${n(rows[0].t_end, 0)} 秒。控制器 0 是 PID，1 是 LQR。`);
out.push('');
out.push('角度单位为度，角速度单位为度每秒，空速单位为 m/s，高度单位为 m，油门为 0 到 1。');
out.push('末段指最后 25 % 的时间窗。换向次数与主频取末 4 秒，主频是除直流外幅值最大的频率。');
out.push('饱和占比指该舵面顶到正负 25 度限位的样本占比，PID 油门那一列指油门顶到 1 的样本占比。');
out.push('');

// Per-case tables
for (const fam of families) {
   const sel = rows.filter((r) => family(r.case_id) === fam);
   out.push(`## ${fam}：纵向与执行机构`);
   out.push('');
   out.push('| id | 控制器 | 工况 | 空速末值 | 空速范围 | 高度末值 | 高度变化率 | 迎角范围 | 俯仰指令 | 俯仰末值 | 俯仰末段误差 |');
   out.push('|---|---|---|---|---|---|---|---|---|---|---|');
   for (const r of sel) {
      out.push(
         `| ${r.case_id} | ${ctrlName(r.ctrl)} | ${r.label} | ${n(r.V_fin)} | ${n(r.V_min)} 到 ${n(r.V_max)} | ` +
            `${n(r.h_fin, 1)} | ${sgn(r.h_rate_end)} | ${n(r.alpha_min)} 到 ${n(r.alpha_max)} | ` +
            `${sgn(r.theta_c_fin)} | ${sgn(r.theta_fin)} | ${sgn(r.ss_e_theta)} |`,
      );
   }
   out.push('');
   out.push('| id | 控制器 | 工况 | 副翼范围 | 副翼末值 | 升降舵范围 | 升降舵末值 | 方向舵范围 | 方向舵末值 | 左油门范围 | 左油门末值 |');
   out.push('|---|---|---|---|---|---|---|---|---|---|---|');
   for (const r of sel) {
      out.push(
         `| ${r.case_id} | ${ctrlName(r.ctrl)} | ${r.label} | ${n(r.da_min, 1)} 到 ${n(r.da_max, 1)} | ${sgn(r.da_fin, 1)} | ` +
            `${n(r.de_min, 1)} 到 ${n(r.de_max, 1)} | ${sgn(r.de_fin, 1)} | ${n(r.dr_min, 1)} 到 ${n(r.dr_max, 1)} | ${sgn(r.dr_fin, 1)} | ` +
            `${n(r.thl_min)} 到 ${n(r.thl_max)} | ${n(r.thl_fin)} |`,
      );
   }
   out.push('');
}

// Lateral and tracking
out.push('## 横航向与跟踪');
out.push('');
out.push('| id | 控制器 | 工况 | 滚转指令 | 滚转末值 | 滚转范围 | p 范围 | r 范围 | 滚转末段误差 | 偏航率末值 | 空速均方根误差 |');
out.push('|---|---|---|---|---|---|---|---|---|---|---|');
for (const r of rows) {
   out.push(
      `| ${r.case_id} | ${ctrlName(r.ctrl)} | ${r.label} | ${sgn(r.phi_c_fin)} | ${sgn(r.phi_fin)} | ` +
         `${n(r.phi_min, 1)} 到 ${n(r.phi_max, 1)} | ${n(r.p_min, 1)} 到 ${n(r.p_max, 1)} | ` +
         `${n(r.r_min, 1)} 到 ${n(r.r_max, 1)} | ${sgn(r.ss_e_phi)} | ${sgn(r.r_max === 0 ? 0 : r.r_min + r.r_max, 1)} | ${n(r.rms_e_V, 3)} |`,
   );
}
out.push('');

// Chatter and envelope
out.push('## 颤振与包线越界');
out.push('');
out.push('| id | 控制器 | 工况 | 升降舵换向 | 副翼换向 | 方向舵换向 | 油门换向 | 升降舵主频 | 油门主频 | q 主频 | 升降舵饱和 | 副翼饱和 | 油门顶满 | 滚转越界 | 俯仰越界 | 空速越界 | 迎角越界 |');
out.push('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|');
for (const r of rows) {
   out.push(
      `| ${r.case_id} | ${ctrlName(r.ctrl)} | ${r.label} | ${r.chat_de} | ${r.chat_da} | ${r.chat_dr} | ${r.chat_thl} | ` +
         `${n(r.f_dom_de, 1)} | ${n(r.f_dom_thl, 1)} | ${n(r.f_dom_q, 1)} | ${pct(r.sat_de)} | ${pct(r.sat_da)} | ${pct(r.sat_thl)} | ` +
         `${pct(r.frac_phi)} | ${pct(r.frac_theta)} | ${pct(r.frac_Vhi)} | ${pct(r.frac_alpha)} |`,
   );
}
out.push('');

// Single-axis cross coupling
out.push('## 单轴满杆时的交叉耦合');
out.push('');
out.push('指令轴以外的两个轴在这一段里的峰值响应，看的是一个轴打满时另外两个轴被带动多少。');
out.push('');
const axisOf = (r) => {
   if (r.roll !== 0) return '滚转';
   if (r.pitch !== 0) return '俯仰';
   return '偏航';
};
out.push('| id | 控制器 | 工况 | 指令轴 | 指令轴末值 | 滚转峰值 | 俯仰峰值 | 偏航率峰值 | 空速末值 |');
out.push('|---|---|---|---|---|---|---|---|---|');
for (const r of rows.filter((x) => x.case_id >= 2 && x.case_id <= 7)) {
   const a = axisOf(r);
   const cmdVal = a === '滚转' ? r.phi_fin : a === '俯仰' ? r.theta_fin : r.q_max;
   out.push(
      `| ${r.case_id} | ${ctrlName(r.ctrl)} | ${r.label} | ${a} | ${sgn(cmdVal)} | ` +
         `${n(Math.max(Math.abs(r.phi_min), Math.abs(r.phi_max)), 2)} | ` +
         `${n(Math.max(Math.abs(r.theta_min), Math.abs(r.theta_max)), 2)} | ` +
         `${n(Math.max(Math.abs(r.r_min), Math.abs(r.r_max)), 2)} | ${n(r.V_fin)} |`,
   );
}
out.push('');

// Controller comparison
out.push('## 同一个工况下两个控制器的对照');
out.push('');
out.push('| id | 工况 | 俯仰末段误差 PID | 俯仰末段误差 LQR | 空速末值 PID | 空速末值 LQR | 升降舵换向 PID | 升降舵换向 LQR | 高度变化率 PID | 高度变化率 LQR |');
out.push('|---|---|---|---|---|---|---|---|---|---|');
const ids = [...new Set(rows.map((r) => r.case_id))].sort((a, b) => a - b);
for (const id of ids) {
   const p = rows.find((r) => r.case_id === id && r.ctrl === 0);
   const l = rows.find((r) => r.case_id === id && r.ctrl === 1);
   if (!p || !l) continue;
   out.push(
      `| ${id} | ${p.label} | ${sgn(p.ss_e_theta)} | ${sgn(l.ss_e_theta)} | ${n(p.V_fin)} | ${n(l.V_fin)} | ` +
         `${p.chat_de} | ${l.chat_de} | ${sgn(p.h_rate_end)} | ${sgn(l.h_rate_end)} |`,
   );
}
out.push('');

// Summary
const summarize = (sel, name) => {
   if (sel.length === 0) return null;
   const maxOf = (k) => Math.max(...sel.map((r) => Math.abs(r[k])));
   const meanOf = (k) => sel.reduce((a, r) => a + r[k], 0) / sel.length;
   return {
      name,
      count: sel.length,
      vLo: Math.min(...sel.map((r) => r.V_min)),
      vHi: Math.max(...sel.map((r) => r.V_max)),
      hRate: meanOf('h_rate_end'),
      thetaErr: meanOf('ss_e_theta'),
      phiPeak: Math.max(...sel.map((r) => Math.max(Math.abs(r.phi_min), Math.abs(r.phi_max)))),
      deChat: meanOf('chat_de'),
      thlChat: meanOf('chat_thl'),
      satDe: meanOf('sat_de'),
      satThl: meanOf('sat_thl'),
      vOut: meanOf('frac_Vhi'),
      alphaOut: meanOf('frac_alpha'),
      alphaPeak: maxOf('alpha_max'),
   };
};
const groupStats = [];
for (const fam of families) {
   for (const c of [0, 1]) {
      const st = summarize(rows.filter((r) => family(r.case_id) === fam && r.ctrl === c), `${fam} ${ctrlName(c)}`);
      if (st) groupStats.push(st);
   }
}
out.push('## 分组汇总');
out.push('');
out.push('| 分组 | 段数 | 空速最低 | 空速最高 | 高度变化率均值 | 俯仰末段误差均值 | 滚转峰值 | 升降舵换向均值 | 油门换向均值 | 升降舵饱和均值 | 油门顶满均值 | 空速越界占比 | 迎角越界占比 | 迎角峰值 |');
out.push('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|');
for (const g of groupStats) {
   out.push(
      `| ${g.name} | ${g.count} | ${n(g.vLo)} | ${n(g.vHi)} | ${sgn(g.hRate)} | ${sgn(g.thetaErr)} | ${n(g.phiPeak, 1)} | ` +
         `${n(g.deChat, 0)} | ${n(g.thlChat, 0)} | ${pct(g.satDe)} | ${pct(g.satThl)} | ${pct(g.vOut)} | ${pct(g.alphaOut)} | ${n(g.alphaPeak)} |`,
   );
}
out.push('');

writeFileSync(outPath, out.join('\n'), 'utf8');
console.log(`段数 ${rows.length}`);
console.log(`写出 ${outPath}`);
