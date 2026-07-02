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

    // Deep high-tech gradient background
    const bgGrad = ctx.createLinearGradient(0, 0, 0, height);
    bgGrad.addColorStop(0, '#060a15');
    bgGrad.addColorStop(1, '#0c1328');
    ctx.fillStyle = bgGrad;
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
    
    // Parse Lidar Cells
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

    // Zoom Button Clicks Listener Hook
    if (!canvas.dataset.zoomHooked) {
        canvas.dataset.zoomHooked = '1';
        canvas.style.cursor = 'pointer';
        canvas.addEventListener('click', (e) => {
            const r = canvas.getBoundingClientRect();
            const clickX = e.clientX - r.left;
            const clickY = e.clientY - r.top;
            
            const btnX = width - 40;
            if (clickX >= btnX && clickX <= btnX + 24) {
                if (clickY >= 20 && clickY <= 44) {
                    // Zoom In (Make span smaller)
                    const currentZoom = window._unifiedMapZoomOverride || _unifiedSpatialMapSpanEma || 16.0;
                    window._unifiedMapZoomOverride = Math.max(4.0, currentZoom - 3.0);
                    localStorage.setItem('_unifiedMapZoomOverride', window._unifiedMapZoomOverride);
                } else if (clickY >= 50 && clickY <= 74) {
                    // Zoom Out (Make span larger)
                    const currentZoom = window._unifiedMapZoomOverride || _unifiedSpatialMapSpanEma || 16.0;
                    window._unifiedMapZoomOverride = Math.min(80.0, currentZoom + 3.0);
                    localStorage.setItem('_unifiedMapZoomOverride', window._unifiedMapZoomOverride);
                } else if (clickY >= 80 && clickY <= 104) {
                    // Auto Zoom Reset
                    window._unifiedMapZoomOverride = null;
                    localStorage.removeItem('_unifiedMapZoomOverride');
                }
            }
        });
    }

    // Initialize/read stored zoom override
    if (window._unifiedMapZoomOverride === undefined) {
        const saved = localStorage.getItem('_unifiedMapZoomOverride');
        window._unifiedMapZoomOverride = saved ? parseFloat(saved) : null;
    }

    // Center calculation (relative to boat if possible)
    let cx = boat ? Number(boat.x_m || 0) : 0;
    let cy = boat ? Number(boat.y_m || 0) : 0;
    let span = 16.0;

    if (window._unifiedMapZoomOverride) {
        span = window._unifiedMapZoomOverride;
    } else {
        // Collect points to determine dynamic bounding box
        const allPts = [];
        for (const p of trail) if (Array.isArray(p) && p.length >= 2) allPts.push([Number(p[0]), Number(p[1])]);
        for (const w of wps) allPts.push([Number(w.x_m || 0), Number(w.y_m || 0)]);
        for (const f of staticFeatures) allPts.push([Number(f.x_m || 0), Number(f.y_m || 0)]);
        if (boat) allPts.push([Number(boat.x_m || 0), Number(boat.y_m || 0)]);

        if (bounds && bounds.min_east_m != null && bounds.max_east_m != null) {
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

        // Limit the span growth to prevent distant noise from ruining the zoom
        let maxPtSpan = 15.0;
        for (const p of allPts) {
            const dist = Math.hypot(p[0] - cx, p[1] - cy);
            if (dist < 32.0) { // Reject extreme noise outliers
                maxPtSpan = Math.max(maxPtSpan, dist * 1.22);
            }
        }
        span = Math.min(45.0, Math.max(maxPtSpan, 12.0));
    }

    // Apply EMA smoothing to the zoom span to make the camera transitions fluid
    if (_unifiedSpatialMapSpanEma == null || !Number.isFinite(_unifiedSpatialMapSpanEma)) {
        _unifiedSpatialMapSpanEma = span;
    } else {
        _unifiedSpatialMapSpanEma = (0.10 * span) + (0.90 * _unifiedSpatialMapSpanEma);
    }
    span = _unifiedSpatialMapSpanEma;

    const scale = Math.min(width, height) / (2.0 * span);
    const toPx = (x, y) => {
        const px = (width * 0.5) + ((x - cx) * scale);
        const py = (height * 0.5) - ((y - cy) * scale);
        return [px, py];
    };

    // 1. Grid (Cyberpunk / Radar Grid)
    const gridStepM = span <= 20 ? 2.0 : (span <= 40 ? 5.0 : 10.0);
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.08)';
    ctx.lineWidth = 1;
    const leftM = cx - span;
    const rightM = cx + span;
    const bottomM = cy - span;
    const topM = cy + span;
    
    // Draw Grid Lines
    let gx = Math.floor(leftM / gridStepM) * gridStepM;
    while (gx <= rightM) {
        const p0 = toPx(gx, bottomM);
        const p1 = toPx(gx, topM);
        ctx.beginPath();
        ctx.moveTo(p0[0], p0[1]);
        ctx.lineTo(p1[0], p1[1]);
        ctx.stroke();
        gx += gridStepM;
    }
    let gy = Math.floor(bottomM / gridStepM) * gridStepM;
    while (gy <= topM) {
        const p0 = toPx(leftM, gy);
        const p1 = toPx(rightM, gy);
        ctx.beginPath();
        ctx.moveTo(p0[0], p0[1]);
        ctx.lineTo(p1[0], p1[1]);
        ctx.stroke();
        gy += gridStepM;
    }

    // Concentric Range Rings (Radar style)
    if (boat) {
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.06)';
        ctx.lineWidth = 1.5;
        const rings = [3.0, 6.0, 10.0, 15.0];
        const boatPx = toPx(boat.x_m, boat.y_m);
        for (const r of rings) {
            if (r < span * 1.5) {
                ctx.beginPath();
                ctx.arc(boatPx[0], boatPx[1], r * scale, 0, Math.PI * 2);
                ctx.stroke();
            }
        }
    }

    // 2. Trail (Glowing Path)
    if (trail.length >= 2) {
        ctx.save();
        ctx.strokeStyle = '#10b981'; // emerald-500
        ctx.lineWidth = 3.5;
        ctx.shadowColor = '#10b981';
        ctx.shadowBlur = 8;
        ctx.beginPath();
        const p0 = toPx(Number(trail[0][0]), Number(trail[0][1]));
        ctx.moveTo(p0[0], p0[1]);
        for (let i = 1; i < trail.length; i++) {
            const pt = trail[i];
            if (Array.isArray(pt) && pt.length >= 2) {
                const q = toPx(Number(pt[0]), Number(pt[1]));
                ctx.lineTo(q[0], q[1]);
            }
        }
        ctx.stroke();
        ctx.restore();
    }

    // 3. Waypoint Path
    if (wps.length >= 2) {
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.22)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        const p0 = toPx(Number(wps[0].x_m || 0), Number(wps[0].y_m || 0));
        ctx.moveTo(p0[0], p0[1]);
        for (let i = 1; i < wps.length; i++) {
            const q = toPx(Number(wps[i].x_m || 0), Number(wps[i].y_m || 0));
            ctx.lineTo(q[0], q[1]);
        }
        ctx.stroke();
        ctx.setLineDash([]);
    }

    // Draw Waypoint Nodes
    for (const w of wps) {
        const q = toPx(Number(w.x_m || 0), Number(w.y_m || 0));
        let color = '#64748b';
        let glow = false;
        if (w.status === 'active') {
            color = '#f59e0b'; // Amber
            glow = true;
        } else if (w.status === 'past') {
            color = '#10b981'; // Green
        } else if (w.status === 'future') {
            color = '#94a3b8'; // Light Slate
        }
        
        ctx.save();
        ctx.fillStyle = color;
        if (glow) {
            ctx.shadowColor = color;
            ctx.shadowBlur = 10;
            ctx.beginPath();
            ctx.arc(q[0], q[1], 7, 0, Math.PI * 2);
            ctx.fill();
        }
        ctx.beginPath();
        ctx.arc(q[0], q[1], 4.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();

        // Label
        ctx.fillStyle = '#94a3b8';
        ctx.font = 'bold 10px monospace';
        ctx.fillText('WP' + ((w.idx != null ? w.idx : 0) + 1), q[0] + 8, q[1] - 4);
    }

    // 4. Lidar Occupancy Grid Cells (Matrix Look)
    const lidarAge = payload.lidar && payload.lidar.age_s != null ? Number(payload.lidar.age_s) : null;
    const lidarStale = Boolean(payload.lidar && payload.lidar.stale);
    let lidarAlpha = 0.65;
    if (lidarStale) lidarAlpha = 0.25;
    else if (lidarAge != null && lidarAge > 0.35) lidarAlpha = 0.35;
    
    const cellSz = 0.20 * scale; // 20cm grid cells mapping
    
    for (const c of lidarCells) {
        const q = toPx(Number(c.x), Number(c.y));
        const conf = Math.max(0, Math.min(1, Number(c.confidence || 0)));
        const persistent = c.persistence === 'persistent';
        
        // Dynamic styling based on occupancy confidence
        let fillStyle;
        if (conf >= 0.72) {
            // High threat (red obstacles)
            fillStyle = `rgba(239, 68, 68, ${lidarAlpha * (0.6 + conf * 0.4)})`;
        } else if (conf >= 0.45) {
            // Warning zones (amber)
            fillStyle = `rgba(245, 158, 11, ${lidarAlpha * (0.4 + conf * 0.3)})`;
        } else {
            // Faint persistent background traces
            fillStyle = persistent ? `rgba(14, 165, 233, ${lidarAlpha * 0.3})` : `rgba(56, 189, 248, ${lidarAlpha * 0.2})`;
        }
        
        ctx.fillStyle = fillStyle;
        ctx.fillRect(q[0] - cellSz * 0.5, q[1] - cellSz * 0.5, cellSz, cellSz);
        
        if (conf >= 0.72) {
            ctx.strokeStyle = `rgba(239, 68, 68, ${lidarAlpha})`;
            ctx.lineWidth = 1;
            ctx.strokeRect(q[0] - cellSz * 0.5, q[1] - cellSz * 0.5, cellSz, cellSz);
        }
    }

    // 5. Obstacle Landmarks (High-confidence target tracking)
    const obstacleLandmarks = Array.isArray(payload.obstacle_landmarks) ? payload.obstacle_landmarks : 
                              (payload.lidar && Array.isArray(payload.lidar.landmarks) ? payload.lidar.landmarks : []);
    
    for (const lm of obstacleLandmarks) {
        const cx_lm = Number(lm.centroid_east_m);
        const cy_lm = Number(lm.centroid_north_m);
        const r_m = Math.max(0.4, Number(lm.radius_m || 0.8));
        const conf = Math.max(0, Math.min(1, Number(lm.confidence || 0)));
        if (!Number.isFinite(cx_lm) || !Number.isFinite(cy_lm)) continue;

        const centerPx = toPx(cx_lm, cy_lm);
        const radiusPx = r_m * scale;
        
        // Soft glowing halo
        ctx.save();
        const haloGrad = ctx.createRadialGradient(centerPx[0], centerPx[1], 1, centerPx[0], centerPx[1], Math.max(6, radiusPx));
        haloGrad.addColorStop(0, `rgba(244, 63, 94, ${conf * 0.35})`); // rose-500
        haloGrad.addColorStop(0.5, `rgba(244, 63, 94, ${conf * 0.12})`);
        haloGrad.addColorStop(1, 'rgba(244, 63, 94, 0)');
        ctx.fillStyle = haloGrad;
        ctx.beginPath();
        ctx.arc(centerPx[0], centerPx[1], Math.max(6, radiusPx), 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();

        // Crosshair ring
        ctx.strokeStyle = `rgba(244, 63, 94, ${conf * 0.65})`;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(centerPx[0], centerPx[1], Math.max(5, radiusPx * 0.7), 0, Math.PI * 2);
        ctx.stroke();

        // Target reticle lines
        ctx.strokeStyle = `rgba(244, 63, 94, ${conf * 0.85})`;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.moveTo(centerPx[0] - 6, centerPx[1]); ctx.lineTo(centerPx[0] + 6, centerPx[1]);
        ctx.moveTo(centerPx[0], centerPx[1] - 6); ctx.lineTo(centerPx[0], centerPx[1] + 6);
        ctx.stroke();
    }

    // 6. Traversable Corridor Wedge & Target Line
    if (boat && corridor && corridor.enabled !== false && corridor.stale !== true) {
        const boatPx = toPx(Number(boat.x_m || 0), Number(boat.y_m || 0));
        
        // Draw green safe sector arc
        const bestDelta = Number(corridor.best_heading_delta_deg || 0);
        const headingDeg = Number(boat.heading_deg || 0);
        const centerRad = (headingDeg - bestDelta) * Math.PI / 180.0 - Math.PI / 2.0;
        const startRad = centerRad - 22 * Math.PI / 180.0;
        const endRad = centerRad + 22 * Math.PI / 180.0;

        ctx.save();
        ctx.fillStyle = 'rgba(16, 185, 129, 0.05)'; // translucent emerald
        ctx.strokeStyle = 'rgba(16, 185, 129, 0.15)';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(boatPx[0], boatPx[1]);
        ctx.arc(
            boatPx[0], boatPx[1],
            corridorLookahead * scale,
            startRad,
            endRad,
            false
        );
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();

        // Draw recommended corridor target line
        const bestEndpoint = corridorEndpoint(bestDelta, corridorLookahead);
        if (bestEndpoint) {
            const q = toPx(bestEndpoint[0], bestEndpoint[1]);
            const score = Math.max(0, Math.min(1, Number(corridor.score || 0)));
            const isOptimal = score >= 0.55;
            
            ctx.save();
            ctx.strokeStyle = isOptimal ? 'rgba(16, 185, 129, 0.85)' : 'rgba(245, 158, 11, 0.85)';
            ctx.lineWidth = 2.5;
            ctx.setLineDash([5, 4]);
            ctx.beginPath();
            ctx.moveTo(boatPx[0], boatPx[1]);
            ctx.lineTo(q[0], q[1]);
            ctx.stroke();
            ctx.setLineDash([]);
            
            // Pulsing target dot
            ctx.fillStyle = isOptimal ? '#10b981' : '#f59e0b';
            ctx.beginPath();
            ctx.arc(q[0], q[1], 4.5, 0, Math.PI * 2);
            ctx.fill();
            
            // Halo around target dot
            ctx.strokeStyle = isOptimal ? 'rgba(16, 185, 129, 0.45)' : 'rgba(245, 158, 11, 0.45)';
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.arc(q[0], q[1], 8 + Math.sin(Date.now() / 180.0) * 2.0, 0, Math.PI * 2);
            ctx.stroke();
            ctx.restore();
        }
    }

    // 7. Static Features (Course Boundary/Target Buoys)
    for (const f of staticFeatures) {
        const q = toPx(Number(f.x_m || 0), Number(f.y_m || 0));
        let color = '#475569';
        let glowColor = '';
        if (f.type === 'boundary_orange') {
            color = '#f97316'; // Orange
            glowColor = 'rgba(249, 115, 22, 0.3)';
        } else if (f.type === 'obstacle_yellow') {
            color = '#eab308'; // Yellow
            glowColor = 'rgba(234, 179, 8, 0.3)';
        } else if (f.type === 'target_buoy') {
            color = '#ef4444'; // Red (correct color target)
            glowColor = 'rgba(239, 68, 68, 0.5)';
        }
        
        ctx.save();
        if (glowColor) {
            ctx.shadowColor = color;
            ctx.shadowBlur = 8;
        }
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(q[0], q[1], f.type === 'target_buoy' ? 5.5 : 4.0, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
    }

    // 8. Boat Rendering (Pointed Hull Shape)
    if (boat) {
        const q = toPx(Number(boat.x_m || 0), Number(boat.y_m || 0));
        const hdg = Number(boat.heading_deg || 0) * Math.PI / 180.0;
        
        ctx.save();
        ctx.translate(q[0], q[1]);
        ctx.rotate(hdg);
        
        // Hull Fill and Outline
        ctx.fillStyle = 'rgba(244, 63, 94, 0.22)';
        ctx.strokeStyle = '#f43f5e'; // rose-500
        ctx.lineWidth = 2.0;
        ctx.beginPath();
        // Modern pointed bow and wider transom
        ctx.moveTo(0, -11);
        ctx.quadraticCurveTo(4.5, -5, 5, 8);
        ctx.lineTo(-5, 8);
        ctx.quadraticCurveTo(-4.5, -5, 0, -11);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();

        // Bow beacon dot
        ctx.fillStyle = '#fb7185';
        ctx.beginPath();
        ctx.arc(0, -11, 2, 0, Math.PI * 2);
        ctx.fill();
        
        // Small heading projection vector
        ctx.strokeStyle = 'rgba(244, 63, 94, 0.55)';
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.moveTo(0, -11);
        ctx.lineTo(0, -26);
        ctx.stroke();
        
        ctx.restore();
    }

    // 9. North Indicator / Compass Rose (Top-Left)
    ctx.save();
    const compX = 35;
    const compY = 40;
    const compR = 15;
    
    // Outer compass ring
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(compX, compY, compR, 0, Math.PI * 2);
    ctx.stroke();
    
    // Compass markers (N, S, E, W tick lines)
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.beginPath();
    ctx.moveTo(compX, compY - compR); ctx.lineTo(compX, compY - compR + 3);
    ctx.moveTo(compX, compY + compR); ctx.lineTo(compX, compY + compR - 3);
    ctx.moveTo(compX - compR, compY); ctx.lineTo(compX - compR + 3, compY);
    ctx.moveTo(compX + compR, compY); ctx.lineTo(compX + compR - 3, compY);
    ctx.stroke();

    // North Needle
    ctx.fillStyle = '#ef4444'; // Red north needle
    ctx.beginPath();
    ctx.moveTo(compX, compY - compR + 1);
    ctx.lineTo(compX - 3.5, compY);
    ctx.lineTo(compX + 3.5, compY);
    ctx.closePath();
    ctx.fill();

    // South Needle
    ctx.fillStyle = '#94a3b8'; // Slate south needle
    ctx.beginPath();
    ctx.moveTo(compX, compY + compR - 1);
    ctx.lineTo(compX - 3.5, compY);
    ctx.lineTo(compX + 3.5, compY);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = '#e2e8f0';
    ctx.font = 'bold 9px var(--sans)';
    ctx.textAlign = 'center';
    ctx.fillText('N', compX, compY - compR - 4);
    
    // ENU Grid scale description
    ctx.fillStyle = 'rgba(148, 163, 184, 0.8)';
    ctx.font = '10px monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`Grid: ${gridStepM}m | Zoom: ${span.toFixed(0)}m`, 60, 42);
    ctx.restore();

    // 10. Zoom Buttons UI Overlay (Top-Right)
    ctx.save();
    const btnX = width - 40;
    const btnW = 24;
    const btnH = 24;
    const isOverride = window._unifiedMapZoomOverride !== null;
    
    const drawBtn = (y, symbol, activeState) => {
        // Base fill
        ctx.fillStyle = 'rgba(15, 23, 42, 0.75)';
        ctx.strokeStyle = activeState ? 'rgba(56, 189, 248, 0.65)' : 'rgba(255, 255, 255, 0.15)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        // Canvas roundRect support
        if (typeof ctx.roundRect === 'function') {
            ctx.roundRect(btnX, y, btnW, btnH, 4);
        } else {
            ctx.rect(btnX, y, btnW, btnH);
        }
        ctx.fill();
        ctx.stroke();
        
        ctx.fillStyle = activeState ? '#38bdf8' : '#e2e8f0';
        ctx.font = 'bold 12px var(--sans)';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(symbol, btnX + btnW / 2, y + btnH / 2);
    };
    
    drawBtn(20, '+', false);
    drawBtn(50, '-', false);
    drawBtn(80, 'A', isOverride);
    ctx.restore();
}
