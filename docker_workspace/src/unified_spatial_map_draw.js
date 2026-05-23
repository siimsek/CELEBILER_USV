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
    const course = payload.course || {};
    const staticFeatures = Array.isArray(course.static_features) ? course.static_features : [];
    const bounds = payload.bounds || null;
    const lidarWorldPts = lidarPts.filter(p => Array.isArray(p) && p.length >= 2 && Number.isFinite(p[0]) && Number.isFinite(p[1]));

    const allPts = [];
    for (const p of trail) if (Array.isArray(p) && p.length >= 2) allPts.push([Number(p[0]), Number(p[1])]);
    for (const w of wps) allPts.push([Number(w.x_m || 0), Number(w.y_m || 0)]);
    for (const p of lidarWorldPts) allPts.push([Number(p[0]), Number(p[1])]);
    for (const f of staticFeatures) allPts.push([Number(f.x_m || 0), Number(f.y_m || 0)]);
    if (boat) allPts.push([Number(boat.x_m || 0), Number(boat.y_m || 0)]);

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
    ctx.fillStyle = `rgba(56,189,248,${lidarAlpha})`;
    for (const p of lidarWorldPts) {
        const q = toPx(Number(p[0]), Number(p[1]));
        ctx.fillRect(q[0] - 1, q[1] - 1, 2, 2);
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
