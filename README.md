
---

# **Parametric Trade‑Off Study of High OPR versus High T4 Design Approaches for a Two‑Spool Turbofan Engine**

**Aero Propulsion Group**  
*Department of Interdisciplinary Technology and Sciences, University of Tehran*  
*Yashar Mohammadi – Student no. 832404024*

---

## **Abstract**

This project presents a thermodynamic performance comparison of two competing gas‑turbine design philosophies: increasing the overall pressure ratio (OPR) through advanced compressor aerodynamics, and raising the turbine inlet temperature (T4) via improved turbine materials and cooling. A two‑spool separate‑flow turbofan rated at 100 kN static sea‑level thrust was modelled using the pyCycle/OpenMDAO framework. A full‑factorial matrix of 129² cases covering OPR from 22 to 30 and T4 from 1300 °C to 1450 °C was simulated. The solver balanced air mass flow, fuel‑to‑air ratio, and shaft powers to satisfy the design constraints. Results show that a 36.4 % increase in OPR decreases thrust‑specific fuel consumption (TSFC) by 6.12 % and raises thermal efficiency by 5.0 %. Conversely, an 11.5 % increase in T4 increases TSFC by 17.5 % and reduces thermal efficiency by 12.5 %, driven by a 21.6 % rise in fuel‑to‑air ratio. Higher T4, however, reduces engine mass flow by 3.4 % and increases specific thrust, offering weight and size advantages. A comparison with the ideal Brayton cycle highlights the cumulative impact of component irreversibilities. The study concludes that, under the imposed fixed‑thrust constraint, the high‑OPR approach offers superior fuel efficiency, whereas the high‑T4 approach benefits engine compactness and weight. The choice between the two philosophies must therefore be guided by the relative priority of mission fuel burn versus engine weight and cost.

---

## **Nomenclature**

| Symbol          | Description                              | Units     |
| --------------- | ---------------------------------------- | --------- |
| OPR             | Overall pressure ratio (fan × LPC × HPC) | –         |
| T4              | Turbine inlet total temperature          | °C        |
| TSFC            | Thrust‑specific fuel consumption         | g/(kN·s)  |
| SFC             | Specific fuel consumption (power basis)  | kg/(kW·h) |
| η<sub>th</sub>  | Thermal efficiency                       | –         |
| ṁ<sub>air</sub> | Air mass flow rate                       | kg/s      |
| ṁ<sub>f</sub>   | Fuel mass flow rate                      | kg/s      |
| FAR             | Fuel‑to‑air ratio                        | –         |
| F/ṁₐᵢᵣ          | Specific thrust                          | kN·s/kg   |
| PR              | Component pressure ratio                 | –         |
| HPC, LPC        | High‑/Low‑pressure compressor            | –         |
| HPT, LPT        | High‑/Low‑pressure turbine               | –         |

---

## **1. Introduction**

Modern civil turbofan engines continuously push the boundaries of efficiency through two primary design pathways. The first, **High OPR**, relies on increasingly sophisticated compressor aerodynamics to achieve ever‑higher cycle pressure ratios, thereby improving the ideal Brayton efficiency. The second, **High T4**, exploits advances in turbine metallurgy and cooling technology to permit higher combustor exit temperatures, raising the mean temperature of heat addition and increasing specific work.  

While both approaches promise lower fuel burn, they impose distinct penalties: high OPR adds compressor stages, weight, and cost, while high T4 demands costly superalloys, intricate cooling, and potentially shortens hot‑section life. This project investigates the trade‑off between the two strategies using a numerical model of a fixed‑thrust two‑spool turbofan, employing the pyCycle library integrated with OpenMDAO. The objective is to map the design space, quantify the sensitivity of key performance metrics, and provide clear guidance on the engineering consequences of each approach.

---

## **2. Engine Model & Simulation Methodology**

### **2.1 Reference Engine Architecture**

A two‑spool separate‑flow turbofan was modelled. The key design parameters are summarised in Table 1.

**Table 1 – Engine design parameters**

| Parameter                                                                                                    | Value                        |
| ------------------------------------------------------------------------------------------------------------ | ---------------------------- |
| Fan pressure ratio                                                                                           | 1.6                          |
| Bypass ratio                                                                                                 | 5.1                          |
| Design thrust (static, sea level)                                                                            | 100 kN                       |
| Component efficiencies (η<sub>fan</sub>, η<sub>LPC</sub>, η<sub>HPC</sub>, η<sub>HPT</sub>, η<sub>LPT</sub>) | 0.92, 0.90, 0.89, 0.90, 0.91 |
| Burner pressure drop (ΔP/P)                                                                                  | 0.08                         |
| Nozzle velocity coefficients (core & bypass)                                                                 | 0.94                         |
| Shaft loss fractions (HP & LP)                                                                               | 0.02                         |
| Design LP / HP spool speeds                                                                                  | 5490 / 15183 rpm             |

### **2.2 Numerical Framework (OpenMDAO / pyCycle)**

The engine was modelled using **pyCycle**, a thermodynamic cycle library built on NASA’s NPSS approach, wrapped in an **OpenMDAO** environment to handle iterative solvers. This permits simultaneous balancing of multiple design variables to satisfy the required performance constraints.

### **2.3 Solver Setup & Balances**

Two independent design inputs are prescribed:
- **T4**, the turbine inlet temperature [°C]
- **OPR**, the overall pressure ratio (fan × LPC × HPC)

The solver performs four major balances:
1. **Compressor pressure ratio split** – the LPC and HPC pressure ratios are distributed to minimize the total number of compressor stages, while ensuring their product (with the fan) equals the target OPR.
   ``` python
    PR_L_MAX = 1.25   # They are derived from 70 years of gas turbine physics,
    PR_H_MAX = 1.38   # specifically the De Haller Diffusion Factor and Mach Number limits.

    self.add_subsystem('stage_estimator', om.ExecComp(
      ['N_total = log(lpc_PR) / log(PR_L_max) + log(hpc_PR) / log(PR_H_max)'],
      lpc_PR={'units': None},
      hpc_PR={'units': None},
      PR_L_max={'val': PR_L_MAX, 'units': None},
      PR_H_max={'val': PR_H_MAX, 'units': None}))

    self.connect('lpc.PR', 'stage_estimator.lpc_PR')
    self.connect('hpc.PR', 'stage_estimator.hpc_PR')

    self.add_objective('stage_estimator.N_total')

    self.add_subsystem('opr_eq', om.ExecComp(
      'hpc_PR = target_opr / (fan_PR * lpc_PR)',
      target_opr={'val': target_opr, 'units': None},
      fan_PR={'units': None},
      lpc_PR={'units': None},
      hpc_PR={'units': None}))

    self.connect('fan.PR', 'opr_eq.fan_PR')
    self.connect('lpc.PR', 'opr_eq.lpc_PR')
    self.connect("opr_eq.hpc_PR", "hpc.PR")
   ```
2. **Thrust balance** – the inlet air mass flow rate $\dot{m}_air$ is varied until the net uninstalled thrust equals 100 kN.
   ```python
    bal.add_balance('W', val=350.0, units='kg/s', eq_units='lbf', lower=10.0, upper=1000.0)
    self.connect('balance.W', 'fc.W')
    self.connect('performance.Fn', 'balance.lhs:W')
    ```
3. **Energy balance** – the fuel-to-air ratio (FAR) is iterated until the turbine inlet temperature T4 reaches the target value.
   ``` python
    bal.add_balance('FAR', val=0.023, units=None, eq_units='degC', lower=0.005, upper=0.05)
    self.connect('balance.FAR', 'burner.Fl_I:FAR')
    self.connect('burner.Fl_O:tot:T', 'balance.lhs:FAR')

    self.add_subsystem('balance', bal)
    self.set_input_defaults('balance.rhs:FAR', target_T4, units='degC')
   ```
4. **Shaft power balance** – HP and LP shaft speeds are held constant; the solver adjusts HPT and LPT pressure ratios so that the power extracted by each turbine exactly matches the power consumed by the respective compressor/fan, accounting for the 2% mechanical loss.
   ``` python
    # HPT Pressure Ratio - for HP shaft power balance
    bal.add_balance('PR_hpt', val=4.0, lower=1.1, upper=20.0, eq_units='hp')
    self.connect('balance.PR_hpt', 'hpt.PR')
    self.connect('hp_shaft.pwr_net', 'balance.lhs:PR_hpt')

    # LPT Pressure Ratio - for LP shaft power balance
    bal.add_balance('PR_lpt', val=4.5, lower=1.1, upper=20.0, eq_units='hp')
    self.connect('balance.PR_lpt', 'lpt.PR')
    self.connect('lp_shaft.pwr_net', 'balance.lhs:PR_lpt')
   ```
   

The code outputs full station data (T, P, s, ν), component pressure ratios, shaft powers, and global performance metrics in a JSON file.


### **2.4 Test Matrix Definition**

To systematically compare the two design approaches, a full‑factorial test matrix was simulated:
- **OPR:** `np.linspace(22, 30, 129)`
- **T4:** `np.linspace(1300, 1450, 129)` °C  

This yields 129² = 16 641 cases, providing a high‑resolution map of the design space.

---
## **3. Results & Discussion**


### **3.1 Overall Performance Contours**

<img src="Plots/Overall%20Performance%20Contours.png" alt="Overall Performance Contours" width="750">

The contour maps present the engine’s response to simultaneous variations in OPR and T4.

**Thrust Specific Fuel Consumption (TSFC)**   
  
The highest TSFC occurs at the lowest OPR and the highest T4, while the minimum is found at the opposite corner—maximum OPR, minimum T4. Quantitatively:
  
- OPR increase → TSFC <span style="background-color:#d4edda; color:#155724; padding:1px 6px; border-radius:4px; font-weight:bold;">– 6.12 %</span> (expanding by 1.00 % with T4)  
- T4 increase → TSFC <span style="background-color:#f8d7da; color:#721c24; padding:1px 6px; border-radius:4px; font-weight:bold;">+ 17.54 %</span> (expanding by 1.26 % with OPR)

  <img src="Plots/TSFC%20vs%20OPR.png" alt="TSFC vs. OPR" width="450">
  <img src="Plots/TSFC%20vs%20T4.png" alt="TSFC vs. T4" width="450">

Thus, under the fixed‑thrust constraint, pushing OPR improves fuel economy, whereaas raising T4 markedly worsens it—an outcome that will be explained by the fuel‑air ratio behavior in Section 3.3.

**Thermal Efficiency (η<sub>th</sub>)**  
Consistent with the TSFC trend, thermal efficiency peaks at maximum OPR and minimum T4.

- OPR increase → η<sub>th</sub> <span style="background-color:#d4edda; color:#155724; padding:1px 6px; border-radius:4px; font-weight:bold;">+ 4.98 %</span> (tightening by 1.12 % across T4)  
- T4 increase → η<sub>th</sub> <span style="background-color:#f8d7da; color:#721c24; padding:1px 6px; border-radius:4px; font-weight:bold;">– 12.54 %</span> (tightening by 0.94 % across OPR)

  <img src="Plots/Thermal Efficiency%20vs%20OPR.png" alt="Thermal Efficiency vs. OPR" width="450">


**Air Mass Flow Rate (ṁ<sub>air</sub>) and Specific Thrust (F/ṁ<sub>air</sub>)**  
  - ṁ<sub>air</sub> is maximised at high OPR, low T4, and minimised at low OPR, high T4.
    - OPR increase → ṁ<sub>air</sub> <span style="background-color:#f8d7da; color:#721c24; padding:1px 6px; border-radius:4px; font-weight:bold;">+ 0.85 %</span> (tightening by 0.23 % across T4)  
    - T4 increase → ṁ<sub>air</sub> <span style="background-color:#d4edda; color:#155724; padding:1px 6px; border-radius:4px; font-weight:bold;">– 3.36 %</span> (tightening by 0.22 % across OPR)
  
    <img src="Plots/Air%20Mass%20Flow%20Rate%20vs%20OPR.png" alt="Air Mass Flow Rate vs. OPR" width="450">

  - Specific thrust (F/ṁ<sub>air</sub>) follows the inverse pattern: it peaks at low OPR, high T4.
    - OPR increase → F/ṁ<sub>air</sub> <span style="background-color:#f8d7da; color:#721c24; padding:1px 6px; border-radius:4px; font-weight:bold;">– 0.84 %</span> (tightening by 0.23 % across T4)  
    - T4 increase → F/ṁ<sub>air</sub> <span style="background-color:#d4edda; color:#155724; padding:1px 6px; border-radius:4px; font-weight:bold;">+ 3.48 %</span> (tightening by 0.24 % across OPR)
  
    <img src="Plots/Specific%20Thrust%20vs%20OPR.png" alt="Specific Thrust vs. OPR" width="450">

These results confirm that high T4 enables a more compact engine (lower mass flow, higher specific thrust), while high OPR has a negligible effect on engine diameter but slightly penalises specific thrust.


---
### **3.2 Component Power Distribution**

**Influence of OPR**  
Increasing OPR from 22 to 30 concentrates the additional aerodynamic loading on the high‑pressure spool:
- HPC power: **+22.62 %**
- HPT power: **+22.30 %**
- Fan, LPC, and LPT power: only **+0.85 %**

<img src="Plots/Fan%20and%20Compressors%20Power%20vs%20OPR.png" alt="Fan and Compressors Power vs OPR" width="750">

**Influence of T4**
Raising T4 from 1300 °C to 1450 °C reduces all component powers by a nearly uniform **3.3–3.4 %** (fan, LPC, HPC, HPT, LPT). This proportional decrease stems from the lower air mass flow; the increase in specific work is insufficient to keep the absolute power levels constant.

<img src="Plots/Fan%20and%20Compressors%20Power%20vs%20T4.png" alt="Fan and Compressors Power vs T4" width="750">

These load shifts have direct structural implications: high OPR demands a heavier, more powerful HP spool, whereas high T4 lightens the entire engine uniformly.

---
### **3.3 Fuel Consumption Details**

Because thrust is held constant at 100 kN, fuel mass flow rate (ṁ<sub>f</sub>) is exactly proportional to TSFC. The observed sensitivities are:
- OPR increase → ṁ<sub>f</sub> <span style="background-color:#d4edda; color:#155724; padding:1px 6px; border-radius:4px; font-weight:bold;">– 6.12 %</span> (expanding by 1.00 % with T4)  
- T4 increase → ṁ<sub>f</sub> <span style="background-color:#f8d7da; color:#721c24; padding:1px 6px; border-radius:4px; font-weight:bold;">+ 17.54 %</span> (expanding by 1.26 % with OPR)

  <img src="Plots/Fuel%20Mass%20Flow%20Rate%20vs%20OPR.png" alt="Fuel Mass Flow Rate vs. OPR" width="450">



The fuel‑to‑air ratio (FAR) reveals the root cause of the T4 penalty:
- OPR increase → FAR <span style="background-color:#d4edda; color:#155724; padding:1px 6px; border-radius:4px; font-weight:bold;">– 6.91 %</span> (expanding by 1.21 % with T4)  
- T4 increase → FAR <span style="background-color:#f8d7da; color:#721c24; padding:1px 6px; border-radius:4px; font-weight:bold;">+ 21.63 %</span> (expanding by 1.58 % with OPR)

  <img src="Plots/FAR%20vs%20OPR.png" alt="FAR vs. OPR" width="450">

Although raising T4 reduces the required air flow by 3.36 %, the FAR must increase by over 21 % to deliver the commanded turbine inlet temperature. The net effect is a disproportionately large fuel flow increase, which overwhelms the specific‑thrust benefit and raises TSFC. In other words, the thermal energy demanded by the higher T4 far outpaces the efficiency gain from a smaller, higher‑specific‑work core when total thrust is fixed.

<img src="Plots/Air%20Mass%20Flow%20Rate%20vs%20T4.png" alt="Air Mass Flow Rate vs. T4" width="450">

---
### **3.4 Comparison with the Ideal Brayton Cycle**

Two extreme design points were benchmarked against an ideal Brayton cycle having the same OPR, T4, and working fluid.

**Case A - High OPR, Moderate T4** (OPR = 30, T4 = 1300 °C)
  
Relative to the ideal cycle, the real engine shows:
<table style="border-collapse: collapse; font-family: Arial, sans-serif; margin: 1em 0;">
  <thead>
    <tr style="background-color: #f2f2f2;">
      <th style="padding: 8px 12px; text-align: left;">Metric</th>
      <th style="padding: 8px 12px; text-align: center;">Deviation from Ideal</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="padding: 6px 12px;">TSFC</td>
      <td style="padding: 6px 12px; text-align: center; background-color: #f8d7da; color: #721c24; font-weight: bold;">
        ⬆ +0.03 %
      </td>
    </tr>
    <tr>
      <td style="padding: 6px 12px;">SFC</td>
      <td style="padding: 6px 12px; text-align: center; background-color: #d4edda; color: #155724; font-weight: bold;">
        ⬇ –4.54 %
      </td>
    </tr>
    <tr>
      <td style="padding: 6px 12px;">η<sub>th</sub></td>
      <td style="padding: 6px 12px; text-align: center; background-color: #d4edda; color: #155724; font-weight: bold;">
        ⬆ +4.75 %
      </td>
    </tr>
    <tr>
      <td style="padding: 6px 12px;">ṁ<sub>air</sub></td>
      <td style="padding: 6px 12px; text-align: center; background-color: #e2e3e5; color: #383d41; font-weight: bold;">
        ⬆ +7.65 %
      </td>
    </tr>
    <tr>
      <td style="padding: 6px 12px;">Specific Thrust</td>
      <td style="padding: 6px 12px; text-align: center; background-color: #f8d7da; color: #721c24; font-weight: bold;">
        ⬇ –7.11 %
      </td>
    </tr>
  </tbody>
</table>
  
<img src="Plots/T-S%20Diagram%20[OPR%20=%2030,%20T4%20=%201300°C].png" alt="T-s Diagram" width="750">

The T‑s diagram shows that compressor irreversibility raises the HPC outlet total temperature by 13.43 % and entropy by 1.39 %. The LPT discharge temperature is 12.11 % lower than ideal, with 1.27 % higher entropy.

<img src="Plots/P-V%20Diagram%20[OPR%20=%2030,%20T4%20=%201300°C].png" alt="P-v Diagram" width="750">

The P‑v diagram indicates that the burner exit total pressure is 9.84 % below ideal, and the cumulative losses result in an LPT exit total pressure 52.67 % lower than ideal.


**Case B - Moderate OPR, High T4** (OPR = 22, T4 = 1450 °C)
  
  Compared to its ideal counterpart, the real engine exhibits:
  <table style="border-collapse: collapse; font-family: Arial, sans-serif; margin: 1em 0;">
  <thead>
    <tr style="background-color: #f2f2f2;">
      <th style="padding: 8px 12px; text-align: left;">Metric</th>
      <th style="padding: 8px 12px; text-align: center;">Deviation from Ideal</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="padding: 6px 12px;">TSFC</td>
      <td style="padding: 6px 12px; text-align: center; background-color: #f8d7da; color: #721c24; font-weight: bold;">
        ⬆ +1.49 %
      </td>
    </tr>
    <tr>
      <td style="padding: 6px 12px;">SFC</td>
      <td style="padding: 6px 12px; text-align: center; background-color: #d4edda; color: #155724; font-weight: bold;">
        ⬇ –2.90 %
      </td>
    </tr>
    <tr>
      <td style="padding: 6px 12px;">η<sub>th</sub></td>
      <td style="padding: 6px 12px; text-align: center; background-color: #d4edda; color: #155724; font-weight: bold;">
        ⬆ +2.98 %
      </td>
    </tr>
    <tr>
      <td style="padding: 6px 12px;">ṁ<sub>air</sub></td>
      <td style="padding: 6px 12px; text-align: center; background-color: #e2e3e5; color: #383d41; font-weight: bold;">
        ⬆ +6.64 %
      </td>
    </tr>
    <tr>
      <td style="padding: 6px 12px;">Specific Thrust</td>
      <td style="padding: 6px 12px; text-align: center; background-color: #f8d7da; color: #721c24; font-weight: bold;">
        ⬇ –6.23 %
      </td>
    </tr>
  </tbody>
</table>
    
<img src="Plots/T-S%20Diagram%20[OPR%20=%2022,%20T4%20=%201450°C].png" alt="T-s Diagram" width="750">

The burner pressure loss again equals 9.84 %, and the LPT exit total pressure is 43.69 % lower. The temperature deficit at the LPT exit is 7.96 %.

<img src="Plots/P-V%20Diagram%20[OPR%20=%2022,%20T4%20=%201450°C].png" alt="P-v Diagram" width="750">

Both cases demonstrate that component inefficiencies (non‑isentropic compression/expansion, burner pressure drop) shift the real cycle significantly away from the ideal, especially at high OPR where the expansion pressure ratio is already high. Nevertheless, the high‑OPR case retains a lower TSFC because the cycle’s thermodynamic advantage outweighs the losses, whereas the high‑T4 case suffers from the severe fuel‑flow penalty discussed above.

### **3.5 Weight, Cost, and Mechanical Complexity Implications**

<img src="Plots/Compressor%20Stages.png" alt="Compressors Stage Numbers" width="450">

The stage‑count estimation indicates that for OPR values above approximately 25, one additional stage is required in the HPC. This, combined with the power distribution findings, leads to the following practical trade‑offs:

**High OPR Approach (22 → 30)**
- *Complexity:* Adds HPC stages, tighter tip clearances, increased seal leakage, and more critical stage matching.
- *Weight:* Extra discs, longer casings, and thicker pressure shells increase engine weight.
- *Cost:* Higher part count, tighter tolerances, and additional blade rows raise both production and maintenance costs.

**High T4 Approach (1300 → 1450 °C)**
- *Complexity:* No additional stages, but extreme demands on turbine cooling—advanced film cooling, thermal barrier coatings (TBCs), and potentially ceramic matrix composites (CMC). The combustor and first HPT nozzle become highly intricate.
- *Weight:* Turbine disc and blades may require denser single‑crystal superalloys, but the reduction in air mass flow (‑3.36 %) and overall engine diameter makes the high‑T4 engine **lighter** than its high‑OPR counterpart.
- *Cost:* Blade manufacturing costs escalate due to single‑crystal casting, cooling hole drilling, and coating processes. However, the smaller engine envelope and lower component count can partly offset these costs. Maintenance costs are likely higher due to the severe thermal environment.

**Summary Comparison**

<table style="border-collapse: collapse; font-family: Arial, sans-serif; margin: 1.5em 0; width: 100%;">
  <thead>
    <tr style="background-color: #f2f2f2;">
      <th style="padding: 10px; text-align: left; width: 30%;">Aspect</th>
      <th style="padding: 10px; text-align: center; width: 35%;">Increasing OPR (22 → 30)</th>
      <th style="padding: 10px; text-align: center; width: 35%;">Increasing T4 (1300 → 1450 °C)</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="padding: 8px; font-weight: bold;">TSFC</td>
      <td style="padding: 8px; text-align: center; background-color: #d4edda; color: #155724;">
        ⬇ –6.12 % <span style="opacity:0.8;">✔</span>
      </td>
      <td style="padding: 8px; text-align: center; background-color: #f8d7da; color: #721c24;">
        ⬆ +17.54 % <span style="opacity:0.8;">✘</span>
      </td>
    </tr>
    <tr>
      <td style="padding: 8px; font-weight: bold;">Thermal efficiency</td>
      <td style="padding: 8px; text-align: center; background-color: #d4edda; color: #155724;">
        ⬆ +4.98 % <span style="opacity:0.8;">✔</span>
      </td>
      <td style="padding: 8px; text-align: center; background-color: #f8d7da; color: #721c24;">
        ⬇ –12.54 % <span style="opacity:0.8;">✘</span>
      </td>
    </tr>
    <tr>
      <td style="padding: 8px; font-weight: bold;">Engine weight</td>
      <td style="padding: 8px; text-align: center; background-color: #f8d7da; color: #721c24;">
        ⬆ Increases <br><small>(more stages)</small>
      </td>
      <td style="padding: 8px; text-align: center; background-color: #d4edda; color: #155724;">
        ⬇ Decreases <br><small>(lower air flow)</small>
      </td>
    </tr>
    <tr>
      <td style="padding: 8px; font-weight: bold;">Specific thrust</td>
      <td style="padding: 8px; text-align: center; background-color: #f8d7da; color: #721c24;">
        ⬇ –0.84 %
      </td>
      <td style="padding: 8px; text-align: center; background-color: #d4edda; color: #155724;">
        ⬆ +3.48 %
      </td>
    </tr>
    <tr>
      <td style="padding: 8px; font-weight: bold;">Mechanical complexity</td>
      <td style="padding: 8px; text-align: center; background-color: #fff3cd; color: #856404;">
        ⬆ Higher <br><small>(aero‑dynamics, more stages)</small>
      </td>
      <td style="padding: 8px; text-align: center; background-color: #fff3cd; color: #856404;">
        ⬆ Higher <br><small>(turbine cooling &amp; materials)</small>
      </td>
    </tr>
    <tr>
      <td style="padding: 8px; font-weight: bold;">Manufacturing cost</td>
      <td style="padding: 8px; text-align: center; background-color: #f8d7da; color: #721c24;">
        ⬆ Rises <br><small>(part count)</small>
      </td>
      <td style="padding: 8px; text-align: center; background-color: #fff3cd; color: #856404;">
        ⬆ Rises <br><small>(material/process),<br>may be offset by smaller size</small>
      </td>
    </tr>
    <tr>
      <td style="padding: 8px; font-weight: bold;">Maintenance cost</td>
      <td style="padding: 8px; text-align: center; background-color: #fff3cd; color: #856404;">
        ⬆ Moderate increase <br><small>(more parts)</small>
      </td>
      <td style="padding: 8px; text-align: center; background-color: #f8d7da; color: #721c24;">
        ⬆ Potentially higher <br><small>(turbine life limits)</small>
      </td>
    </tr>
  </tbody>
</table>

---

## **4. Conclusions**

This study systematically compared the High OPR and High T4 design philosophies for a 100 kN two‑spool separate‑flow turbofan at sea‑level static conditions. The key findings are:

1. **Fuel efficiency:** Within the examined range, a 36.4 % increase in OPR reduces TSFC by 6.12 % and raises thermal efficiency by 5.0 %. In contrast, an 11.5 % increase in T4 **increases** TSFC by 17.5 % and **lowers** thermal efficiency by 12.5 %, driven by a 21.6 % rise in fuel‑to‑air ratio. The fixed‑thrust constraint magnifies the fuel‑flow penalty of high T4.

2. **Engine size and weight:** Higher T4 reduces air mass flow by 3.36 % and raises specific thrust by 3.48 %, yielding a more compact, lighter engine. Higher OPR adds HPC stages and structural weight with negligible reduction in diameter.

3. **Shaft load distribution:** High OPR concentrates additional loading on the HP spool (+22.6 % HPC/HPT power), while high T4 reduces all component powers proportionally (‑3.3 %). These differences dictate structural and bearing design requirements.

4. **Real‑cycle penalties:** Comparison with the ideal Brayton cycle reveals that component losses reduce LPT exit total pressure by up to 53 %. The high‑OPR case retains a lower TSFC, indicating that the thermodynamic cycle benefit outweighs the cumulative losses.

5. **Design guidance:** If the primary objective is minimum fuel burn, the high‑OPR, moderate‑T4 strategy is clearly superior within the studied envelope. If, however, engine weight, frontal area, or acquisition cost are paramount, the high‑T4, moderate‑OPR philosophy becomes attractive despite its fuel‑consumption penalty, owing to the smaller engine envelope and reduced component count.

Future work should incorporate validated weight and direct operating cost models, investigate the sensitivity of the T4 trend to component efficiencies (particularly HPT and HPC), and extend the design space to include variations in fan pressure ratio and bypass ratio for a fully optimised comparison.

---

## **References**

1. OpenMDAO Development Team, *OpenMDAO: An Open‑Source Framework for Multidisciplinary Design, Analysis, and Optimization*, 2024.  
2. J. J. Alonso et al., *pyCycle: A Thermodynamic Cycle Modeling Library for Propulsion System Design*, NASA/TM–2018-219956.  
3. P. P. Walsh and P. Fletcher, *Gas Turbine Performance*, 2nd ed., Blackwell Science, 2004.  
4. J. D. Mattingly, *Elements of Gas Turbine Propulsion*, McGraw‑Hill, 1996.

---

## **Appendix**

The complete set of simulation results (16 641 cases) is available as a structured JSON file. The Python code used for the pyCycle/OpenMDAO model and the post‑processing scripts can be provided upon request. Additional figures showing the sensitivity of HPC and HPT pressure ratios to OPR and T4 are omitted for brevity but can be included if needed.

---
