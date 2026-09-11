# Idea & Technology Proposal: Draft (based on wifi_hybrid_try_2.cc project)

> Draft answers to `Idea_Technology_Proposal_Template.docx`, grounded in what the
> NS-3 hybrid-network project (Phase 1, PIC Lab / KIT) can actually substantiate.
> Items marked **[NEEDS INPUT]** require facts this repo doesn't contain
> (applicant names, market data, business decisions). Do not submit those as-is.

---

## Proposal title
**RSSI/PDR-Based Automatic WiFi Mesh-Cellular Failover for Multi-Robot
Communication at Construction Sites**
(working title, adjust to house style)

## Applicant / Team
**[NEEDS INPUT]**: applicant name(s) and affiliation. The reporting group is
PIC Lab, KIT; confirm the individual applicant names before submission.

---

## 1. Idea Description

**Key focus: Why is this idea / technology needed?**

**Overview.** This project builds a self-switching network system for
multi-robot teams working on construction sites. Robots on a site, such as
survey drones, materials-handling units, and safety-monitoring robots, need to
talk to each other and to a central controller constantly and reliably. Our
idea is a "hybrid" network that automatically moves each robot's connection
between a local WiFi mesh and a cellular network (LTE, Long Term Evolution,
or 5G), whichever is working better at that moment, without the robot ever losing its connection or
needing to manually reconnect.

**Background.** Construction sites are increasingly deploying several robots
that must work together rather than independently: for example a survey drone,
a materials robot, and a safety-monitoring unit all coordinating on the same
job. For robots to act as one team instead of separate machines, they need to
stay in near-real-time sync, which is why this project targets an end-to-end
synchronization delay of less than 200ms. We started this project because no single
network technology available on a construction site can guarantee that on its
own. A WiFi mesh is fast and cheap to deploy but loses signal behind concrete
walls, steel framing, and heavy equipment; cellular (LTE/5G) reaches everywhere
on-site but is slower and more costly to rely on all the time. Rather than pick
one, we set out to test whether a system could automatically use WiFi when it's
strong and fall back to cellular when it's not, and prove that the switch
itself happens fast enough that the robot fleet never loses coordination. We
tested this first through a detailed NS-3 (Network Simulator 3) network
simulation (Phase 1, PIC Lab
research completed by March 2026) rather than building physical hardware right
away, so the switching logic could be proven and tuned safely and cheaply
before committing to expensive real-robot field trials.

![Top-down view of a multi-robot construction site. Four WiFi mesh access points provide overlapping coverage, but concrete structures create dead zones. A robot that moves into a dead zone loses its WiFi link and is automatically switched to the LTE/5G cellular fallback, keeping it connected.](site_scenario.png)

*Figure 1. The problem the technology addresses. On a real site, obstacles such
as concrete structures block WiFi coverage in unpredictable places. Robots A and
B stay on the WiFi mesh, while robot C, shadowed by a central structure, is
automatically moved to the cellular fallback so it never loses coordination with
the fleet.*

**Problem solved.** The technology solves the "seamless failover" problem for
multi-robot construction connectivity: keeping every robot continuously and
reliably connected to the network and to each other as physical obstacles,
distance from the WiFi mesh, and radio interference change moment to moment on
a busy job site, without robots losing coordination or needing manual network
reconfiguration.

**Target customers.** Construction robotics companies and general contractors
who deploy multiple robots on the same site and need them to operate as one
coordinated system; and makers of robot network hardware ("edge gateways") who
need this automatic-switching capability built into their products.

**Product direction.** The near-term output is the switching decision engine
itself, proven in simulation, designed to be built into the network hardware
("gateway") each robot already carries. The longer-term direction is to turn
this into a stand-alone product: either software licensed to companies that
build robot gateways, or a network-reliability service that construction
companies can add to any job site's robot fleet.

---

## 2. Implementation: Technology & Service

**Core technology:** a hybrid path controller that fuses two independent
trigger signals, EWMA-smoothed (Exponentially Weighted Moving Average) WiFi
RSSI (Received Signal Strength Indicator, α=0.6) and a 1-second sliding-window
packet delivery ratio (PDR), with hysteresis (return threshold = trigger
threshold + 3dB, requiring 2 consecutive good checks) to prevent oscillation,
plus a staleness detector (forces failover if no WiFi frame heard for over 3s,
catching the case where RSSI degrades too gradually for threshold logic to
notice). Path changes are executed as **make-before-break routing** (a new
IPv4 (Internet Protocol version 4) host route is installed before the old one
is removed), so an active TCP (Transmission Control Protocol) session survives
the switch without reconnecting. This keeps interruption to the millisecond
range instead of forcing a full socket timeout and retry.

**Main features and service structure:** from the operator's point of view, the
system delivers a small set of concrete capabilities:
- **Always-connected robots.** Each robot keeps a single, continuous network
  identity; the system decides which underlying link (WiFi or cellular) carries
  its traffic at any moment, with no manual reconnection.
- **Automatic, seamless failover.** When WiFi quality degrades, traffic moves to
  cellular, and back to WiFi when it recovers, without dropping the active
  session, keeping service interruption in the millisecond range.
- **Per-robot connection assurance.** Every switch event is measured and logged,
  so the fleet operator can see, per robot, how reliably the coordination-latency
  target is being met.
- **Technology-agnostic fallback.** The same architecture supports either LTE or
  5G as the fallback link, so it fits sites with different cellular coverage.

The capability is structured to be delivered in two complementary forms: an
embedded switching module built into the network gateway each robot already
carries, and an operator-facing connectivity-assurance layer that turns the
per-switch measurements into fleet-level reliability reporting.

**System architecture:** a 4-node 802.11s WiFi mesh backbone with dual-band
hotspot APs (access points; 5GHz 802.11ac / 2.4GHz 802.11n, shared SSID
[Service Set Identifier] for roaming) serving robot clients, backed by a single
macro LTE eNB (evolved Node B) or 5G gNB (next-generation Node B)
whose EPC (Evolved Packet Core) gateway shares the same service IP as the WiFi
path, so failover is a routing change,
not a different destination. Propagation uses a building-aware propagation loss
model applied consistently across WiFi, LTE, and 5G against a fixed 7-building,
400×400m site layout, so the comparison between technologies is like-for-like.

![Hybrid WiFi mesh and cellular failover architecture. The robot client keeps a single service IP; the hybrid path controller continuously fuses WiFi signal strength and packet-delivery measurements to decide whether traffic flows over the primary WiFi mesh or the cellular fallback, while a measurement layer records every switch for fleet-level reliability reporting.](architecture_diagram.png)

*Figure 2. System architecture. The robot client holds a single, stable service
identity. The hybrid path controller sits between the client and the two access
technologies and moves traffic between the primary WiFi mesh and the cellular
fallback. Because both paths reach the same service address, a handover is an
internal routing change rather than a reconnection to a new destination.*

**How a handover happens (step by step).** The controller runs a short periodic
evaluation cycle for every robot:
1. **Sense.** It reads the smoothed WiFi signal strength and the recent
   packet-delivery ratio for that robot, and checks when the last WiFi frame was
   heard.
2. **Decide.** It compares these against the trigger thresholds. If WiFi has
   weakened past the threshold, delivery has dropped, or no WiFi frame has been
   heard for several seconds, the robot is a candidate to move to cellular. To
   return to WiFi, the signal must recover past a higher threshold and stay there
   for consecutive checks.
3. **Act.** When a switch is warranted, the controller installs the new path
   before removing the old one (make-before-break), so the robot's active session
   is never left without a route.
4. **Record.** The controller timestamps the switch, measures how long service
   was interrupted, and logs the event for per-robot reliability reporting.

![Signal-over-time view of an automatic handover. As WiFi signal strength falls below the trigger threshold the robot is switched to the cellular fallback, and once it recovers above a higher return threshold the robot is switched back to WiFi. The gap between the two thresholds is the hysteresis margin that prevents rapid back-and-forth switching.](handover_timeline.png)

*Figure 3. A single handover cycle. Traffic moves to cellular when WiFi drops
below the trigger threshold and returns to WiFi only after the signal recovers
above a higher return threshold and stays there, so the robot does not oscillate
between links while the signal hovers near the edge.*

The following table summarizes the switching decision logic:

| Condition observed | Controller decision | Purpose |
|---|---|---|
| WiFi signal below trigger threshold, or delivery degraded | Move traffic to cellular fallback | Preserve connectivity before the link fails |
| No WiFi frame heard for several seconds | Force move to cellular | Catch gradual signal loss that thresholds miss |
| WiFi signal recovers above the higher return threshold for consecutive checks | Move traffic back to WiFi | Return to the preferred link only once it is genuinely stable |
| Signal fluctuating near the threshold | Hold current path (hysteresis) | Prevent rapid back-and-forth switching |

![State-machine view of the switching logic. The system is in one of two states, on WiFi or on cellular, and transitions between them only when the signal, delivery, and staleness conditions are met, with make-before-break routing ensuring the active session always has a valid path during the transition.](switching_state_machine.png)

*Figure 4. The controller as a two-state machine. Transitions are asymmetric by
design: leaving WiFi is triggered by any of several degradation conditions,
while returning requires a stronger, sustained recovery, which biases the system
toward the preferred WiFi link without allowing unstable switching.*

**Differentiation and technical strengths:**
- Generic multipath solutions such as Multipath TCP (MPTCP) do not reason about
  signal strength and delivery jointly, nor apply hysteresis tuned to a specific
  propagation environment. This project's controller is validated against real
  vendor access-point hardware profiles (TP-Link EAP225, Netgear Orbi 960, ASUS
  ZenWiFi XT8) rather than idealized radios.
- It produces a *quantified* per-switch service-interruption metric (measured in
  milliseconds), not just aggregate throughput. That is the number a robotics
  customer actually needs against a less-than-200ms coordination budget.
- It evaluates LTE and 5G in the *same* fallback role and the *same* topology,
  a direct head-to-head comparison that few public studies provide for a
  construction-robotics use case.

**Current implementation stage:** proof-of-concept / simulation validation.
Phase 1 (through Mar 2026) is complete: an NS-3.45 comparison of WiFi Mesh+LTE
vs. WiFi Mesh+5G technology across multiple random seeds and node
configurations, establishing the switching controller and the
performance-measurement pipeline. Subsequent work has extended the system to a
higher-density scenario of up to 20 mobile robot clients, in which the
controller performs automatic switching in both directions, including reliable
cellular-to-WiFi return, while every client continues to meet the
less-than-200ms end-to-end coordination-latency target. Additional simulation
runs are planned to strengthen the statistical basis of the mode comparison.
This is **not yet** a hardware field trial.

---

## 3. Commercialization Potential & Market

**Target market and customer segments.** The technology serves three connected
segments along the construction-connectivity value chain:
- **Primary: construction robotics fleet operators** (general contractors and
  specialist automation firms) who run multiple robots on one site and need them
  to operate as a coordinated system rather than as isolated machines.
- **Secondary: robot and gateway equipment manufacturers** (Original Equipment
  Manufacturers) who need a connectivity-assurance capability built into the
  network gateway their robots already carry.
- **Enabling: telecom operators and private-network integrators** deploying
  private LTE/5G at construction and industrial sites, for whom automatic
  WiFi-cellular failover is a value-adding feature of their offering.

**Market size and demand.** The idea sits at the intersection of two
fast-growing markets, both driven by the same underlying forces: acute
construction-labor shortages, jobsite-safety pressure, and the Industry 4.0 push
toward automated, always-connected operations.

| Market | 2025 | 2026 | Forecast | CAGR | Source |
|---|---|---|---|---|---|
| Construction robotics | USD 6.55 B | USD 7.79 B | USD 15.39 B by 2030 | ~18.6% | The Business Research Company, 2026 |
| Private 5G network | USD 3.89 B | USD 5.88 B | USD 150.66 B by 2033 | ~58.9% | Grand View Research, 2026 |

*CAGR (Compound Annual Growth Rate) is the average year-over-year growth rate a
market would need to expand from its starting value to its forecast value over
the given period.*

Estimates vary with how analysts scope each sector (narrow robotic-systems
definitions run smaller, while broad definitions that include integrated
industrial machinery run into the hundreds of billions), but every major source
agrees on sustained double-digit growth. On the demand side, the labor gap is
concrete: the United States construction sector alone was estimated to need on
the order of 546,000 additional workers in 2023, with comparable shortages in
Germany, Japan, and South Korea, which is a direct economic driver for
multi-robot deployment. As sites deploy more autonomous machines, reliable
multi-robot coordination becomes a prerequisite rather than an optional feature,
and that is precisely the problem this technology addresses.

*Figures are drawn from third-party market reports (The Business Research
Company / Grand View Research, 2026) and are cited for context; a formal
addressable-market estimate for the specific WiFi-cellular failover niche should
be commissioned before commercial launch.*

**Business model and revenue generation.** Revenue is generated through three
complementary streams that can be phased in as the technology matures:
- **Technology / IP licensing.** License the switching controller (the
  RSSI/PDR fusion algorithm and its make-before-break routing) to robot and
  gateway manufacturers on a per-unit royalty or annual license-fee basis.
- **Connectivity-assurance subscription.** Offer site operators a recurring
  (per-site or per-robot, per-month) service that turns the per-switch
  measurement pipeline into fleet-level reliability dashboards and reporting.
- **Integration and technology-transfer fees.** One-time engineering and
  technology-transfer engagements with telecom operators and system integrators
  embedding the capability into private-network offerings.

Early revenue is expected from licensing and integration engagements, with the
subscription service providing recurring revenue as deployments scale.

**Competitive landscape and competitive advantage.** The relevant competitors
fall into two groups: proprietary heterogeneous-network stacks from telecom and
networking vendors (private 5G combined with WiFi-offload), and general-purpose
open multipath tooling such as Multipath TCP (MPTCP). Both were designed for
generic enterprise or consumer connectivity rather than for coordinated robot
fleets on obstruction-heavy sites. This technology's advantages are:
- **Joint RSSI and delivery-based switching, environment-tuned.** Unlike
  generic multipath, the controller reasons about signal strength and packet
  delivery together and applies hysteresis tuned to a specific radio
  environment, and it has been validated against real vendor access-point
  hardware profiles rather than idealized radios.
- **A coordination-latency guarantee that competitors do not target.** The
  system is built and measured against a less-than-200ms end-to-end
  robot-coordination budget, with a quantified per-switch service-interruption
  metric, which is the number a robotics customer actually needs and which
  general-purpose failover products do not report.
- **Technology-neutral fallback.** The same architecture works with either LTE
  or 5G, so it fits sites with different cellular coverage without redesign.

**Sources.** Market figures and demand indicators in this section are drawn from
the following third-party reports (2026 editions; accessed July 2026):
1. Construction robotics market size and CAGR: The Business Research Company,
   *Construction Robotics Global Market Report 2026*
   (https://www.giiresearch.com/report/tbrc1619533-construction-robotics-global-market-report.html;
   also distributed via https://www.researchandmarkets.com/reports/6035262/construction-robotics-market-report).
2. Private 5G network market size and CAGR: Grand View Research,
   *Private 5G Network Market Size, Industry Report, 2033*
   (https://www.grandviewresearch.com/industry-analysis/private-5g-network-market).
3. Construction labor-shortage and automation demand indicators: IMARC Group,
   *Construction Robots Market Report*
   (https://www.imarcgroup.com/construction-robots-market).

---

## 4. Future Plans

The technology follows a staged development path that moves from validated
simulation toward laboratory testing and, ultimately, construction-site field
deployment and commercialization.

![Development and commercialization roadmap showing five stages: a completed simulation foundation, enhanced simulation, laboratory validation, field validation, and commercialization. The project is currently between the foundation and enhanced-simulation stages.](roadmap_timeline.png)

*Figure 5. Staged roadmap from validated simulation to commercialization. The
foundation stage is complete; work is currently advancing into the
enhanced-simulation stage, with laboratory and field validation and
commercialization to follow.*

**Completed foundation.** The first development stage delivered a validated
simulation of the automatic WiFi+LTE / WiFi+5G technology switching controller,
establishing the core switching logic and the performance-measurement pipeline.

**Planned technical development.** The next stage raises the realism of the
model so that its results transfer reliably to physical environments. Four
improvements are planned:
- **Realistic mobility modeling.** Replace random node movement with structured
  construction-site movement patterns (patrol loops, path movement between
  obstacles, and work stops with dwell time) so that signal-variation and
  switching behavior match real robot operation.
- **Per-flow Quality of Service (QoS) traffic modeling.** Separate robot control
  commands, sensor telemetry, and video streams into distinct flows so that
  latency-critical control traffic can be measured on its own against the
  latency target, rather than through an aggregate average.
- **WiFi mesh internal handover modeling.** Explicitly model access-point
  boundary crossings within the WiFi mesh and add a guard mechanism, so that
  brief in-mesh handovers are not mistaken for cellular switching events.
- **Statistical reinforcement.** Expand the number of simulation runs and apply
  confidence-interval estimation and significance testing, so that performance
  differences between the WiFi+LTE and WiFi+5G technology modes can be reported with
  statistical rigor.

**Laboratory validation.** Following the enhanced simulation, the switching
controller will be validated against a laboratory testbed to confirm behavior
under real radio and hardware conditions before moving to a live site.

**Field validation and deployment.** The final technical stage is validation on
an operating construction site, confirming seamless failover and the
coordination-latency target under real-world obstacles, mobility, and
interference.

**Commercialization pathway (1 to 3 years).** Candidate routes include
protecting the RSSI/PDR fusion switching method through intellectual-property
filing, licensing the switching controller to robot-gateway manufacturers and
telecom system integrators, and offering a connectivity-assurance service built
on the project's performance-measurement pipeline. The specific pathway, timing,
and partnership model will be selected as the technology matures through
laboratory and field validation.

---

## 5. Revisions & Supplementary Content

This is submitted as a new proposal. If any element of this idea was previously
submitted to a past ICT Challenge event, summarize here what has changed since
that submission; otherwise this section may be left blank.

---

## What still needs your input before this is submission-ready
1. Applicant name(s) and affiliation.
2. Confirm/refresh the cited market figures (Section 3) against the latest report
   editions, and, if the competition requires it, commission a formal
   addressable-market estimate for the specific WiFi-cellular failover niche.
3. Final commercialization-pathway decision (licensing vs. subscription vs.
   spinout emphasis; Sections 3-4).
4. Confirm whether this is a new submission or a revision of a prior one
   (Section 5).
