# InnoHub Special Award — Competition Rules

**EBiM Competition 2026**
**Version 1.0 · 27 August 2026**

Sponsored by InnoHub by TÜV Rheinland.

---

## EN

### 1. Scope and eligibility

**Task 3 (Assisted Living & Feeding) only.** Task 1 and Task 2 entries are not in scope.

Open to teams that have advanced to Phase II with a Task 3 entry, and to those teams only.
Teams that submitted a valid Phase I entry but did not advance are not eligible. A team
holding entries in more than one task qualifies through its Task 3 entry and is evaluated
on that entry alone. Teams without an assigned testbed site or slot are included: the
evaluation is simulation-only and no site presence is required.

### 2. What is evaluated, and by whom

The evaluation is run by the organizers, in simulation, on the Phase II runnable Task 3
policy every Phase II team was required to submit — regardless of that team's Phase I
submission pathway.

Organizers enable the `plus` flag in the evaluation configuration. This is an
organizer-side switch. **Teams do not need to edit any configuration, submit anything
additional, or opt in.** There is no deadline for teams.

Policies must run autonomously. Keyboard, pedal, and GELLO teleoperation interfaces exist
in the simulation stack; a submitted policy that depends on any of them for human input
during the run is not a valid entry for this award.

There is no real-robot component. This award does not use, affect, or depend on any
Phase II physical testbed session.

### 3. Scoring

Total **10 points** = Illumination Robustness **4 points** + Collision Avoidance
**6 points**.

Task 3 comprises four stages: Table Setup → Feeding → Bean Recovery → Cleanup. Points are
awarded per stage, against the Task 3 stage success criteria defined in the
[Rulebook](https://ebim-benchmark.github.io/docs/Autonomous_Robot_Benchmark_Rulebook_1.0.pdf),
where Task 3 is referred to as Track 3.

### 4. Illumination Robustness — 4 points

Four lighting configurations, **1 point each**, scored at **0.25 points per successful
stage**.

| Profile | Description | Maximum |
|---|---|---|
| Low lux | 150 lx ≤ ambient ≤ 500 lx | 1 pt |
| High lux | 1000 lx ≤ ambient ≤ 1500 lx | 1 pt |
| Gradient illuminance 1 | 150 lx → 500 lx → 150 lx, simulating a lighting change | 1 pt |
| Gradient illuminance 2 | 1000 lx → 1500 lx → 1000 lx, simulating a lighting change | 1 pt |

Subtotal: the sum of the four configurations, to a maximum of 4 points.

### 5. Collision Avoidance — 6 points

Three obstacles are placed on the ground along the task route, **2 points each**, scored
at **0.5 points per stage**. A stage scores only if the task stage succeeds **and** no
contact occurs with that obstacle during the stage.

| Obstacle | Type | Maximum |
|---|---|---|
| Cable | Stationary, deformable | 2 pts |
| Book | Stationary, rigid | 2 pts |
| Ball | Initially stationary, rigid; may roll after contact | 2 pts |

Subtotal: the sum of the three obstacles, to a maximum of 6 points.

### 6. Contact

**Definition.** Contact means any non-zero contact registered between an obstacle and
either (a) any robot link, or (b) any task object — plate, cup, bowl, or spoon — whether
that object is currently held or not.

Beans that have left the bowl or the spoon are excluded. Contact between a loose bean and
an obstacle does not zero a stage.

**Effect.** Contact with an obstacle during a stage zeroes that obstacle's score for that
stage (0.5 points). It does not affect the other two obstacles, does not affect any other
stage, and does not affect the illumination subtotal. There is no additional or escalated
collision penalty.

**Reset.** All three obstacles are reset to their initial poses at the start of each
stage. Collision scoring is therefore independent across stages — an obstacle displaced
during one stage does not carry into the next.

### 7. Ranking

Single ranking of all eligible Task 3 entries, by total score descending, using standard
competition ranking.

There is no tie-break. Teams with the same total score receive the same rank, and the next
rank is advanced by the number of tied teams: two teams tied at 1st are both ranked 1st,
and the next team is ranked 3rd. Teams with a total score of 0 are not ranked.

Scores are published as computed, without rounding.

### 8. Prizes

| Rank | Prize |
|---|---|
| 1st | USD 300 |
| 2nd | USD 200 |
| 3rd | USD 100 |

One set of prize positions overall, not per task.

A prize is awarded only where a team holds that rank, and every team holding a
prize-winning rank receives that rank's prize in full. A rank left vacant by a tie is not
awarded, and neither is a rank unfilled because too few teams scored above zero.

*Worked example.* Scores of 10.0, 10.0, 9.5, 9.5 and 0 produce ranks 1, 1, 3, 3 and one
unranked entry. Two teams receive USD 300 each; 2nd place is vacant and no USD 200 is
awarded; two teams receive USD 100 each.

This award is separate from the AMD Simulation Prize, which carries the same amounts but
is awarded per task on different criteria. A team may win both.

### 9. Independence and results

Scoring is entirely independent of the main leaderboards and does not affect main task
rankings. No ground-truth-use multiplier and no Phase I multiplier applies. A team may win
both a main-track prize and this award.

Results are announced on **1 October 2026**, together with the main competition results.

### 10. Standard reference

> The lighting evaluation is designed with reference to IEC 62849; however, it assesses
> performance only in a simulated environment. Real-world test results have not yet been
> obtained, and this evaluation should not be considered equivalent to passing the lighting
> assessment specified in the standard.

---

## 中文

### 1. 适用范围与参赛资格

**仅限任务三（辅助生活与喂食）。** 任务一、任务二不在本奖项范围内。

仅面向以任务三参赛条目晋级 Phase II 的队伍。已提交有效 Phase I 方案但未晋级的队伍不具备资格。同时拥有多项任务条目的队伍，以其任务三条目取得资格，并仅就该条目接受评测。无指定站点或时段的队伍同样纳入：本评测仅在仿真环境进行，无需现场参与。

### 2. 评测对象与执行方

评测由主办方于仿真环境中执行，对象为每支 Phase II 队伍必须提交的 Phase II 任务三可运行策略，与该队伍 Phase I 的提交路径无关。

主办方在评测配置中启用 `plus` 标志。该标志由主办方一侧控制，**队伍无需修改任何配置、无需额外提交、无需报名**，亦无任何截止时间。

策略必须自主运行。仿真环境中存在键盘、踏板与 GELLO 遥操作接口；若提交的策略在运行过程中依赖上述任一接口接受人工输入，则不构成本奖项的有效参赛条目。

本奖项不含实机环节，不使用、不影响、也不依赖任何 Phase II 实体测试平台时段。

### 3. 评分

总分 **10 分** = 光照鲁棒性 **4 分** + 避障能力 **6 分**。

任务三包含四个 stage：摆桌 → 喂食 → 豆粒回收 → 收拾。评分以[规则手册](https://ebim-benchmark.github.io/docs/Autonomous_Robot_Benchmark_Rulebook_1.0.pdf)所定义的任务三各 stage 成功判据为准（该手册中任务三称为 Track 3），逐 stage 计分。

### 4. 光照鲁棒性 — 4 分

四组光照配置，**每组 1 分**，每成功维持一个 stage 得 **0.25 分**。

| 光照配置 | 参数描述 | 满分 |
|---|---|---|
| 低照度 | 150 lx ≤ 环境光 ≤ 500 lx | 1 分 |
| 高照度 | 1000 lx ≤ 环境光 ≤ 1500 lx | 1 分 |
| 渐变照度 1 | 由 150 lx 渐变至 500 lx，再渐变回 150 lx，模拟灯光变化过程 | 1 分 |
| 渐变照度 2 | 由 1000 lx 渐变至 1500 lx，再渐变回 1000 lx，模拟灯光变化过程 | 1 分 |

小计：四组光照配置得分之和，满分 4 分。

### 5. 避障能力 — 6 分

三种障碍物置于任务行动路线的地面，**每项 2 分**，每个 stage 计 **0.5 分**。该 stage 须任务成功**且**未与该障碍物发生接触，方可得分。

| 障碍物 | 类型 | 满分 |
|---|---|---|
| 线缆 | 静态、可变形 | 2 分 |
| 书本 | 静态、刚性 | 2 分 |
| 球 | 初始静止、刚性；受接触后可能滚动 | 2 分 |

小计：三项避障得分之和，满分 6 分。

### 6. 接触

**定义。** 接触指障碍物与下列任一者之间发生的任何非零接触：(a) 机器人任一连杆；(b) 任一任务物件（盘、杯、碗、汤匙），无论该物件当下是否被持握。

已脱离碗或汤匙的豆粒不计入：散落豆粒与障碍物之接触不导致该 stage 失分。

**效力。** 于某 stage 中与障碍物发生接触时，该障碍物在该 stage 不得分（0.5 分）。此判罚不影响其余两项障碍物，不影响其他任何 stage，亦不影响光照小计。不设额外或升级的碰撞判罚。

**重置。** 三项障碍物于每个 stage 开始时重置至初始位姿。因此避障计分在各 stage 之间彼此独立——某一 stage 中被推移的障碍物不会延续至下一 stage。

### 7. 排名

所有符合资格的任务三参赛条目统一排名，按总分由高至低，采用标准竞赛排名法。

不设并列决胜规则。总分相同之队伍并列同一名次，其后名次依并列队伍数顺延：两队并列第一名者，两队皆为第一名，下一队为第三名。总分为 0 之队伍不予排名。

成绩依实际计算结果公布，不作四舍五入。

### 8. 奖金

| 名次 | 奖金 |
|---|---|
| 第一名 | USD 300 |
| 第二名 | USD 200 |
| 第三名 | USD 100 |

全场共一套奖项名次，非按任务分设。

奖金仅于该名次确有队伍时发放；凡居于获奖名次之队伍，均完整获得该名次之奖金。因并列而从缺之名次不予发放；因得分高于 0 之队伍不足而未产生之名次，亦不予发放。

*示例。* 得分为 10.0、10.0、9.5、9.5 与 0 时，名次为第一、第一、第三、第三，另一队不予排名。两队各得 USD 300；第二名从缺，不发放 USD 200；两队各得 USD 100。

本奖项与 AMD 仿真奖为两个独立奖项。该奖金额相同，但按任务分设且评判标准不同。同一队伍可同时获得两者。

### 9. 独立性与结果公布

本奖项评分完全独立于主赛道排行榜，不影响主任务排名。不适用任何 ground-truth 使用倍率，亦不适用任何 Phase I 倍率。同一队伍可同时获得主赛道奖项与本奖项。

结果将于 **2026 年 10 月 1 日**与主赛道最终结果一并公布。

### 10. 标准引用说明

> 光照评测参照 IEC 62849 标准设计，但仅考核仿真环境下的表现，暂未获取实测表现，不等同于通过标准设计的光照考核。
