function risk=Risk(risk_score, n)
riskPercent = risk_score / n;  % n is the number of the parkingslot data
if riskPercent <= 0.15
    risk=10;
elseif riskPercent <= 0.20
    risk=6;
elseif riskPercent <= 0.25
    risk=4;
elseif riskPercent <= 0.33
    risk=2;
else
    risk=1;
end