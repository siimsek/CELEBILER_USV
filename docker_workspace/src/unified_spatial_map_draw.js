let _unifiedSpatialMapSpanEma = null;

function drawUnifiedSpatialMap(canvas, payload) {
    if (!canvas || !payload) return;
    const rect = canvas.getBoundingClientRect();
    const width = Math.max(320, Math.floor(rect.width || 0));
    const height = Math.max(240, Math.floor(rect.height || 0));
    if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width;
        canvas.height = height;
    }
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.fillStyle = '#050812';
    ctx.fillRect(0, 0, width, height);

    const boat = payload.boat || null;
    const trail = (payload.trail && Array.isArray(payload.trail.points)) ? payload.trail.points : [];
    const wps = Array.isArray(payload.waypoints) ? payload.waypoints : [];
    const lidarPts = (payload.lidar && Array.isArray(payload.lidar.points)) ? payload.lidar.points : [];
    const lidarCellsRaw = (payload.lidar && Array.isArray(payload.lidar.occupancy_cells)) ? payload.lidar.occupancy_cells : [];
    const course = payload.course || {};
    const staticFeatures = Array.isArray(course.static_features) ? course.static_features : [];
    const bounds = payload.bounds || null;
    const lidarWorldPts = lidarPts.filter(p => Array.isArray(p) && p.length >= 2 && Number.isFinite(p[0]) && Number.isFinite(p[1]));
    const lidarCells = [];
    for (const c of lidarCellsRaw) {
        if (!c || typeof c !== 'object') continue;
        const x = Number(c.x);
        const y = Number(c.y);
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        let confidence = Number(c.confidence);
        if (!Number.isFinite(confidence)) {
            const stable = Math.max(0, Number(c.stable_count || 0));
            const hits = Math.max(0, Number(c.hit_count || 0));
            confidence = Math.min(1, (stable / 8.0) * 0.55 + (hits / 12.0) * 0.45);
        }
        lidarCells.push({
            x,
            y,
            confidence: Math.max(0, Math.min(1, confidence)),
            stable_count: Math.max(0, Number(c.stable_count || 0)),
            hit_count: Math.max(0, Number(c.hit_count || 0)),
            persistence: String(c.persistence || '')
        });
    }
    const corridor = payload.traversable_corridor || {};
    const corridorCandidates = Array.isArray(corridor.candidates) ? corridor.candidates : [];
    const corridorLookahead = Number(corridor.lookahead_m || 7.0);
    const corridorEndpoint = (deltaDeg, distanceM) => {
        if (!boat) return null;
        const headingDeg = Number(boat.heading_deg || 0);
        const bearingRad = (headingDeg - Number(deltaDeg || 0)) * Math.PI / 180.0;
        return [
            Number(boat.x_m || 0) + (Math.sin(bearingRad) * Number(distanceM || corridorLookahead)),
            Number(boat.y_m || 0) + (Math.cos(bearingRad) * Number(distanceM || corridorLookahead))
        ];
    };

    const allPts = [];
    for (const p of trail) if (Array.isArray(p) && p.length >= 2) allPts.push([Number(p[0]), Number(p[1])]);
    for (const w of wps) allPts.push([Number(w.x_m || 0), Number(w.y_m || 0)]);
    for (const p of lidarWorldPts) allPts.push([Number(p[0]), Number(p[1])]);
    for (const c of lidarCells) allPts.push([Number(c.x), Number(c.y)]);
    const obstacleLandmarks = Array.isArray(payload.obstacle_landmarks) ? payload.obstacle_landmarks : 
                              (payload.lidar && Array.isArray(payload.lidar.landmarks) ? payload.lidar.landmarks : []);
    for (const lm of obstacleLandmarks) {
        if (lm && Number.isFinite(Number(lm.centroid_east_m)) && Number.isFinite(Number(lm.centroid_north_m))) {
            allPts.push([Number(lm.centroid_east_m), Number(lm.centroid_north_m)]);
        }
    }
    for (const f of staticFeatures) allPts.push([Number(f.x_m || 0), Number(f.y_m || 0)]);
    if (boat) allPts.push([Number(boat.x_m || 0), Number(boat.y_m || 0)]);
    if (boat && corridor && corridor.stale !== true) {
        for (const c of corridorCandidates) {
            const p = corridorEndpoint(Number(c.heading_delta_deg || 0), corridorLookahead);
            if (p) allPts.push(p);
        }
        const best = corridorEndpoint(Number(corridor.best_heading_delta_deg || 0), corridorLookahead);
        if (best) allPts.push(best);
    }

    let cx = boat ? Number(boat.x_m || 0) : 0;
    let cy = boat ? Number(boat.y_m || 0) : 0;
    let span = 20.0;

    if (bounds && bounds.min_east_m != null && bounds.max_east_m != null && bounds.min_north_m != null && bounds.max_north_m != null) {
        const minE = Number(bounds.min_east_m);
        const maxE = Number(bounds.max_east_m);
        const minN = Number(bounds.min_north_m);
        const maxN = Number(bounds.max_north_m);
        cx = (minE + maxE) * 0.5;
        cy = (minN + maxN) * 0.5;
        span = Math.max((maxE - minE) * 0.55, (maxN - minN) * 0.55, 8.0);
    } else if (!boat && allPts.length) {
        let sx = 0, sy = 0;
        for (const p of allPts) { sx += p[0]; sy += p[1]; }
        cx = sx / allPts.length;
        cy = sy / allPts.length;
    }

    for (const p of allPts) {
        span = Math.max(span, Math.hypot(p[0] - cx, p[1] - cy) * 1.25);
    }
    span = Math.max(span, 15.0);
    if (_unifiedSpatialMapSpanEma == null || !Number.isFinite(_unifiedSpatialMapSpanEma)) {
        _unifiedSpatialMapSpanEma = span;
    } else {
        _unifiedSpatialMapSpanEma = (0.15 * span) + (0.85 * _unifiedSpatialMapSpanEma);
    }
    span = _unifiedSpatialMapSpanEma;

    const scale = Math.min(width, height) / (2.0 * span);
    const toPx = (x, y) => {
        const px = (width * 0.5) + ((x - cx) * scale);
        const py = (height * 0.5) - ((y - cy) * scale);
        return [px, py];
    };

    const gridStepM = span <= 30 ? 2.0 : 5.0;
    ctx.strokeStyle = 'rgba(110,130,170,0.22)';
    ctx.lineWidth = 1;
    const leftM = cx - span;
    const rightM = cx + span;
    const bottomM = cy - span;
    const topM = cy + span;
    let gx = Math.floor(leftM / gridStepM) * gridStepM;
    while (gx <= rightM) {
        const p0 = toPx(gx, bottomM);
        const p1 = toPx(gx, topM);
        ctx.beginPath(); ctx.moveTo(p0[0], p0[1]); ctx.lineTo(p1[0], p1[1]); ctx.stroke();
        gx += gridStepM;
    }
    let gy = Math.floor(bottomM / gridStepM) * gridStepM;
    while (gy <= topM) {
        const p0 = toPx(leftM, gy);
        const p1 = toPx(rightM, gy);
        ctx.beginPath(); ctx.moveTo(p0[0], p0[1]); ctx.lineTo(p1[0], p1[1]); ctx.stroke();
        gy += gridStepM;
    }

    for (const f of staticFeatures) {
        const q = toPx(Number(f.x_m || 0), Number(f.y_m || 0));
        if (f.type === 'boundary_orange') ctx.strokeStyle = 'rgba(251,146,60,0.45)';
        else if (f.type === 'obstacle_yellow') ctx.strokeStyle = 'rgba(250,204,21,0.45)';
        else if (f.type === 'target_buoy') ctx.strokeStyle = 'rgba(248,113,113,0.45)';
        else ctx.strokeStyle = 'rgba(148,163,184,0.35)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(q[0], q[1], f.type === 'boundary_orange' ? 3.5 : 2.5, 0, Math.PI * 2);
        ctx.stroke();
    }

    const lidarAge = payload.lidar && payload.lidar.age_s != null ? Number(payload.lidar.age_s) : null;
    const lidarStale = Boolean(payload.lidar && payload.lidar.stale);
    let lidarAlpha = 0.65;
    if (lidarStale) lidarAlpha = 0.35;
    else if (lidarAge != null && lidarAge > 0.35) lidarAlpha = 0.35;
    if (lidarCells.length > 0) {
        for (const c of lidarCells) {
            const q = toPx(Number(c.x), Number(c.y));
            const conf = Math.max(0, Math.min(1, Number(c.confidence || 0)));
            const persistent = c.persistence === 'persistent';
            const radius = Math.max(1.5, Math.min(6.0, 1.4 + (conf * 4.4)));
            const alpha = Math.max(0.16, Math.min(0.92, lidarAlpha * (0.30 + conf * 0.90)));
            ctx.fillStyle = persistent ? `rgba(14,165,233,${alpha})` : `rgba(56,189,248,${alpha})`;
            ctx.beginPath();
            ctx.arc(q[0], q[1], radius, 0, Math.PI * 2);
            ctx.fill();
            if (conf >= 0.72) {
                ctx.strokeStyle = `rgba(186,230,253,${Math.min(0.75, alpha + 0.10)})`;
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.arc(q[0], q[1], radius + 1.0, 0, Math.PI * 2);
                ctx.stroke();
            }
        }
    } else {
        ctx.fillStyle = `rgba(56,189,248,${lidarAlpha})`;
        for (const p of lidarWorldPts) {
            const q = toPx(Number(p[0]), Number(p[1]));
            ctx.fillRect(q[0] - 1, q[1] - 1, 2, 2);
        }
    }

    // ----------------------------------------------------
    // Draw Obstacle Landmarks
    // ----------------------------------------------------
    if (obstacleLandmarks && obstacleLandmarks.length > 0) {
        for (const lm of obstacleLandmarks) {
            const cx_lm = Number(lm.centroid_east_m);
            const cy_lm = Number(lm.centroid_north_m);
            const px_lm = Number(lm.peak_east_m);
            const py_lm = Number(lm.peak_north_m);
            const r_m = Math.max(0.5, Number(lm.radius_m || 1.0));
            const conf = Math.max(0, Math.min(1, Number(lm.confidence || 0)));
            
            if (!Number.isFinite(cx_lm) || !Number.isFinite(cy_lm)) continue;

            // 1. Draw Heatmap/Density Halo
            const centerPx = toPx(cx_lm, cy_lm);
            const radiusPx = r_m * scale;
            
            const grad = ctx.createRadialGradient(centerPx[0], centerPx[1], 1, centerPx[0], centerPx[1], Math.max(5, radiusPx));
            grad.addColorStop(0, `rgba(239, 68, 68, ${conf * 0.4})`);      // rose-500
            grad.addColorStop(0.5, `rgba(244, 63, 94, ${conf * 0.15})`);
            grad.addColorStop(1, 'rgba(239, 68, 68, 0)');
            
            ctx.fillStyle = grad;
            ctx.beginPath();
            ctx.arc(centerPx[0], centerPx[1], Math.max(5, radiusPx), 0, Math.PI * 2);
            ctx.fill();

            // 2. Draw Landmark Center (Centroid)
            ctx.fillStyle = '#f43f5e'; // rose-500
            ctx.beginPath();
            ctx.arc(centerPx[0], centerPx[1], 3.0, 0, Math.PI * 2);
            ctx.fill();
            
            // 3. Draw Peak Point
            if (Number.isFinite(px_lm) && Number.isFinite(py_lm)) {
                const peakPx = toPx(px_lm, py_lm);
                ctx.fillStyle = '#facc15'; // amber-400
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.arc(peakPx[0], peakPx[1], 2.5, 0, Math.PI * 2);
                ctx.fill();
                ctx.stroke();
            }
        }
    }

    if (boat && corridor && corridor.enabled !== false && corridor.stale !== true) {
        const boatPx = toPx(Number(boat.x_m || 0), Number(boat.y_m || 0));
        for (const c of corridorCandidates) {
            const endpoint = corridorEndpoint(Number(c.heading_delta_deg || 0), corridorLookahead);
            if (!endpoint) continue;
            const q = toPx(endpoint[0], endpoint[1]);
            const score = Math.max(0, Math.min(1, Number(c.score || 0)));
            ctx.strokeStyle = `rgba(34,197,94,${0.08 + score * 0.20})`;
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(boatPx[0], boatPx[1]);
            ctx.lineTo(q[0], q[1]);
            ctx.stroke();
        }
        const bestDelta = Number(corridor.best_heading_delta_deg || 0);
        const bestEndpoint = corridorEndpoint(bestDelta, corridorLookahead);
        if (bestEndpoint) {
            const q = toPx(bestEndpoint[0], bestEndpoint[1]);
            const score = Math.max(0, Math.min(1, Number(corridor.score || 0)));
            ctx.strokeStyle = score >= 0.55 ? 'rgba(34,197,94,0.95)' : 'rgba(245,158,11,0.9)';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(boatPx[0], boatPx[1]);
            ctx.lineTo(q[0], q[1]);
            ctx.stroke();
            ctx.fillStyle = score >= 0.55 ? 'rgba(34,197,94,0.95)' : 'rgba(245,158,11,0.95)';
            ctx.beginPath();
            ctx.arc(q[0], q[1], 4, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    if (trail.length >= 2) {
        ctx.strokeStyle = 'rgba(34,197,94,0.95)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        const p0 = toPx(Number(trail[0][0]), Number(trail[0][1]));
        ctx.moveTo(p0[0], p0[1]);
        for (let i = 1; i < trail.length; i++) {
            const pt = trail[i];
            if (!Array.isArray(pt) || pt.length < 2) continue;
            const q = toPx(Number(pt[0]), Number(pt[1]));
            ctx.lineTo(q[0], q[1]);
        }
        ctx.stroke();
    }

    if (wps.length >= 2) {
        ctx.strokeStyle = 'rgba(255,255,255,0.35)';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        const p0 = toPx(Number(wps[0].x_m || 0), Number(wps[0].y_m || 0));
        ctx.moveTo(p0[0], p0[1]);
        for (let i = 1; i < wps.length; i++) {
            const q = toPx(Number(wps[i].x_m || 0), Number(wps[i].y_m || 0));
            ctx.lineTo(q[0], q[1]);
        }
        ctx.stroke();
    }
    for (const w of wps) {
        const q = toPx(Number(w.x_m || 0), Number(w.y_m || 0));
        let color = '#94a3b8';
        if (w.status === 'active') color = '#f59e0b';
        else if (w.status === 'past') color = '#22c55e';
        else if (w.status === 'future') color = '#e2e8f0';
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(q[0], q[1], 4, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = '#cbd5e1';
        ctx.font = '11px monospace';
        ctx.fillText(String((w.idx != null ? w.idx : 0) + 1), q[0] + 6, q[1] - 6);
    }

    if (boat) {
        const q = toPx(Number(boat.x_m || 0), Number(boat.y_m || 0));
        const hdg = Number(boat.heading_deg || 0) * Math.PI / 180.0;
        ctx.save();
        ctx.translate(q[0], q[1]);
        ctx.rotate(-hdg + Math.PI / 2.0);
        ctx.fillStyle = '#fb7185';
        ctx.beginPath();
        ctx.moveTo(0, -9);
        ctx.lineTo(6, 7);
        ctx.lineTo(-6, 7);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
    }

    const northBase = toPx(cx, cy + Math.min(span * 0.85, span));
    const northTip = toPx(cx, cy + Math.min(span * 0.85, span) + 4.0);
    ctx.strokeStyle = 'rgba(248,250,252,0.85)';
    ctx.fillStyle = 'rgba(248,250,252,0.85)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(northBase[0], northBase[1]);
    ctx.lineTo(northTip[0], northTip[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(northTip[0], northTip[1]);
    ctx.lineTo(northTip[0] - 4, northTip[1] + 6);
    ctx.lineTo(northTip[0] + 4, northTip[1] + 6);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = 'rgba(203,213,225,0.8)';
    ctx.font = '12px monospace';
    ctx.fillText('ENU local  E+ / N+', 10, 16);
}
