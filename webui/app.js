const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];
const API={ key:localStorage.getItem('apiKey')||'DEVKEY123', headers(){return {'x-api-key':this.key}}, async get(p){const r=await fetch(p,{headers:this.headers()}); if(!r.ok) throw await r.json(); return r.json()}, async post(p,b){const r=await fetch(p,{method:'POST',headers:{...this.headers(),'content-type':'application/json'},body:JSON.stringify(b||{})}); if(!r.ok) throw await r.json(); return r.json()}, async patch(p,b){const r=await fetch(p,{method:'PATCH',headers:{...this.headers(),'content-type':'application/json'},body:JSON.stringify(b||{})}); if(!r.ok) throw await r.json(); return r.json()}, async del(p){const r=await fetch(p,{method:'DELETE',headers:this.headers()}); if(!r.ok) throw await r.json(); return r.json()}, };

function rssiToQuality(rssi){
  if(rssi===null || rssi===undefined) return null;
  const v = Math.max(-100, Math.min(-30, rssi));
  return Math.round(((v + 100) / 70) * 100); // 0..100
}
function wifiBarsHTML(rssi){
  const q = rssiToQuality(rssi);
  if(q===null) return '<div class="wifi none" title="sin datos"></div>';
  const bars = Math.ceil(q/25);
  return `<div class="wifi bars-${bars}" title="RSSI ${rssi} dBm / ${q}%">
    <span></span><span></span><span></span><span></span>
  </div>`;
}
function parseTimestamp(ts){
  if(ts===null || ts===undefined) return null;
  const n = typeof ts === 'number' ? ts : parseFloat(ts);
  if(!Number.isFinite(n)) return null;
  return n > 1e12 ? n : n * 1000;
}
function formatLastSeen(ts){
  const ms = parseTimestamp(ts);
  if(!ms) return '—';
  try{
    return new Date(ms).toLocaleString();
  }catch(_){
    return '—';
  }
}
function formatDuration(seconds){
  if(seconds===null || seconds===undefined) return '—';
  const n = typeof seconds === 'number' ? seconds : parseFloat(seconds);
  if(!Number.isFinite(n) || n <= 0) return '—';
  const parts=[];
  let remaining=Math.floor(n);
  const days=Math.floor(remaining/86400); remaining%=86400;
  const hours=Math.floor(remaining/3600); remaining%=3600;
  const minutes=Math.floor(remaining/60); remaining%=60;
  const secs=remaining;
  if(days) parts.push(days+'d');
  if(hours) parts.push(hours+'h');
  if(minutes) parts.push(minutes+'m');
  if(!parts.length || secs) parts.push(secs+'s');
  return parts.join(' ');
}
function toast(msg,k='ok',trace=null){ const t=$('#toast'); t.innerHTML=(k==='ok'?'✅ ':'⚠️ ')+msg+(trace?`<br><small>trace:${trace}</small>`:''); t.classList.add('show'); setTimeout(()=>t.classList.remove('show'),2600); }
function hz(on,off){const p=on+off;return p?1000.0/p:0} function duty(on,off){const p=on+off;return p?100.0*on/p:0}
const Dash={
  async refresh(){
    try{
      const r=await API.get('/api/state');
      const st=r.state,cfg=r.cfg;
      const fmt=(v,d=1,zeroOk=false)=>{
        if(typeof v!=='number' || !Number.isFinite(v)) return '--';
        if(v<=0 && !zeroOk) return '--';
        return v.toFixed(d);
      };
      $('#modeLbl').textContent=st.mode;
      const pumpLbl=$('#pumpLbl');
      pumpLbl.textContent=st.actuator.pump_on?'ON':'OFF';
      pumpLbl.className='badge '+(st.actuator.pump_on?'ok':'');
      Manual.setButton(!!st.actuator.pump_on);
      $('#onms').textContent=st.actuator.on_ms;
      $('#offms').textContent=st.actuator.off_ms;
      $('#pulses').textContent=st.actuator.pulses_total;
      $('#runtime').textContent=(st.actuator.runtime_ms_total/1000).toFixed(1)+' s';
      $('#tankPct').textContent=st.tank.filtered_pct.toFixed(1);
      $('#hopperPct').textContent=st.hopper.filtered_pct.toFixed(1);
      $('#tankGauge').style.width=st.tank.filtered_pct+'%';
      $('#hopperGauge').style.width=st.hopper.filtered_pct+'%';
      $('#hzLbl').textContent=hz(st.actuator.on_ms,st.actuator.off_ms).toFixed(2);
      $('#dutyLbl').textContent=duty(st.actuator.on_ms,st.actuator.off_ms).toFixed(1);
      $('#onInput').value=st.actuator.on_ms;
      $('#offInput').value=st.actuator.off_ms;
      $('#onVal').textContent=st.actuator.on_ms;
      $('#offVal').textContent=st.actuator.off_ms;
      const model=st.auto_model||{};
      $('#emaRun').textContent=fmt(model.ema_run_s,1);
      $('#slope').textContent=fmt(model.ema_slope_pct_s,3);
      $('#inertia').textContent=fmt(model.ema_inertia_s,1,true);
      $('#overshoot').textContent=fmt(model.ema_overshoot_pct,2,true);
      Auto.setButton(st.mode==='AUTO');
      $('#targetPct').value=cfg.hopper.cal.target_pct;
      $('#hystPct').value=cfg.hopper.cal.hysteresis_pct;
      $('#spillGuard').value=cfg.auto.anti_spill_margin_pct;
      $('#tankLowLock').value=cfg.auto.tank_low_lock_pct;
      Auto.setTankSensor(cfg.auto.use_tank_sensor!==false);
      const dashAuto=$('#dashAutoToggle');
      const dashManual=$('#dashManualToggle');
      if(dashAuto && dashManual){
        const isAuto=st.mode==='AUTO';
        dashAuto.textContent=isAuto?'Detener automático':'Iniciar automático';
        dashAuto.classList.toggle('on',isAuto);
        dashManual.textContent=Manual.on?'Apagar bomba':'Encender bomba';
        dashManual.disabled=isAuto;
        dashManual.title=isAuto?'Disponible sólo en modo manual':'';
        dashManual.classList.toggle('on',Manual.on && !isAuto);
      }
      Alerts.process(st,cfg);
    }catch(e){
      toast('Error leyendo estado. Verifica API Key (DEVKEY123).','err',e.detail||e.trace);
    }
  }
};
const Manual={on:false,setButton(on){this.on=!!on;const b=$('#btnManual'); b.classList.toggle('on',this.on); b.setAttribute('aria-pressed',this.on?'true':'false'); b.querySelector('.label').textContent=this.on?'Apagar':'Encender';},async toggle(){const desired=!this.on; try{const r=await API.post('/api/pump',{value:desired?'ON':'OFF',ttl_ms:30000,reason:'manual'}); this.setButton(desired); await Dash.refresh(); toast(desired?'Bomba encendida':'Bomba apagada','ok',r.trace);}catch(e){toast('No se pudo cambiar bomba','err',e.detail||e.trace)}},async applyDuty(){const on_ms=parseInt($('#onInput').value||'100',10); const off_ms=parseInt($('#offInput').value||'233',10); try{const r=await API.post('/api/actuator/duty',{on_ms,off_ms}); toast('Duty aplicado','ok',r.trace); await Dash.refresh();}catch(e){toast('No se pudo aplicar duty','err',e.detail||e.trace)}}};
const Auto={
  on:false,
  tankSensor:true,
  setButton(on){
    this.on=!!on;
    const b=$('#btnAuto');
    b.classList.toggle('on',this.on);
    b.querySelector('.label').textContent=this.on?'Parar':'Iniciar';
  },
  setTankSensor(enabled){
    this.tankSensor=!!enabled;
    const b=$('#tankSensorToggle');
    if(!b) return;
    b.classList.toggle('on',!this.tankSensor);
    b.setAttribute('aria-pressed',this.tankSensor?'false':'true');
    b.textContent=this.tankSensor?'Ignorar sensor tanque':'Usar sensor tanque';
    b.title=this.tankSensor?'El automático se bloqueará si el tanque está bajo':'El automático ignorará el sensor del tanque';
  },
  async toggle(){
    try{
      const r=await API.post('/api/mode/'+(this.on?'MANUAL_OFF':'AUTO'));
      this.setButton(!this.on);
      await Dash.refresh();
      toast('Modo cambiado','ok',r.trace);
    }catch(e){toast('No se pudo cambiar modo','err',e.trace)}
  },
  async apply(){
    const target=parseFloat($('#targetPct').value||'65');
    const hyst=parseFloat($('#hystPct').value||'5');
    const guard=parseFloat($('#spillGuard').value||'1.2');
    const lock=parseFloat($('#tankLowLock').value||'15');
    try{
      const r=await API.patch('/api/config',{hopper:{cal:{target_pct:target,hysteresis_pct:hyst}}, auto:{anti_spill_margin_pct:guard,tank_low_lock_pct:lock}});
      toast('Automático actualizado','ok',r.trace);
      await Dash.refresh();
    }catch(e){toast('No se pudo actualizar AUTO','err',e.trace)}
  },
  async toggleTankSensor(){
    const desired=!this.tankSensor;
    try{
      const r=await API.patch('/api/config',{auto:{use_tank_sensor:desired}});
      await Dash.refresh();
      toast(desired?'Sensor de tanque habilitado':'Sensor de tanque ignorado','ok',r.trace);
    }catch(e){
      toast('No se pudo actualizar sensor tanque','err',e.trace||e.detail);
    }
  }
};
const Calibration={async refresh(){try{const r=await API.get('/api/state'); const cfg=r.cfg; $('#tankEmpty').value=cfg.tank.cal.empty_mm; $('#tankFull').value=cfg.tank.cal.full_mm; $('#hopEmpty').value=cfg.hopper.cal.empty_mm; $('#hopFull').value=cfg.hopper.cal.full_mm;}catch(e){toast('No se pudo leer calibración','err',e.trace)}},async save(){const te=parseInt($('#tankEmpty').value||'800',10); const tf=parseInt($('#tankFull').value||'200',10); const he=parseInt($('#hopEmpty').value||'800',10); const hf=parseInt($('#hopFull').value||'200',10); if(!(te>tf)) return toast('Tanque: Vacío debe ser mayor a Lleno','err'); if(!(he>hf)) return toast('Tolva: Vacío debe ser mayor a Lleno','err'); try{const r=await API.patch('/api/config',{tank:{cal:{empty_mm:te,full_mm:tf}}, hopper:{cal:{empty_mm:he,full_mm:hf}}}); toast('Calibración guardada','ok',r.trace); await Dash.refresh();}catch(e){toast('Error guardando calibración','err',e.detail||e.trace)}}};
const Profiles={async refresh(){try{const r=await API.get('/api/profiles'); const box=$('#profilesList'); box.innerHTML=''; (r.profiles||[]).forEach(p=>{const div=document.createElement('div'); div.className='item'; div.innerHTML=`<span class="name">${p.name}</span><span class="badge">${p.on_ms}/${p.off_ms} ms</span><span><button class="ghost" data-apply="${p.name}">Aplicar</button><button class="ghost" data-del="${p.name}">Borrar</button></span>`; box.appendChild(div);}); box.onclick=async ev=>{const b=ev.target.closest('button'); if(!b) return; if(b.dataset.apply){try{await API.post('/api/profiles/apply/'+b.dataset.apply,{}); toast('Perfil aplicado'); await Dash.refresh();}catch(e){toast('Error al aplicar','err',e.trace)}} else if(b.dataset.del){try{await API.del('/api/profiles/'+b.dataset.del); toast('Perfil eliminado'); await Profiles.refresh();}catch(e){toast('Error al eliminar','err',e.trace)}}};}catch(e){toast('No se pudieron leer perfiles','err',e.trace)}}, async save(){const name=($('#profName').value||'').trim(); const on_ms=parseInt($('#profOn').value||'100',10); const off_ms=parseInt($('#profOff').value||'233',10); if(!name) return toast('Pon un nombre de perfil','err'); try{await API.post('/api/profiles',{name,on_ms,off_ms}); toast('Perfil guardado'); await Profiles.refresh();}catch(e){toast('No se pudo guardar perfil','err',e.trace)}}};

const Boards={
  async refresh(){
    try{
      const r = await API.get('/api/boards');
      const box = $('#boardsList');
      const data = Array.isArray(r) ? r : (r.boards || r.items || []);
      data.sort((a,b)=>{
        if(!!a.online === !!b.online){
          return (a.name||a.board_id||'').localeCompare(b.name||b.board_id||'');
        }
        return a.online ? -1 : 1;
      });
      if(!data.length){
        box.innerHTML = '<div class="card glass board empty"><p>No hay equipos registrados.</p></div>';
        return;
      }
      box.innerHTML = data.map(b=>{
        const status = b.online ? 'online' : 'offline';
        const statusText = b.online ? 'En línea' : 'Desconectado';
        const wifi = wifiBarsHTML(b.rssi);
        const last = formatLastSeen(b.last_seen);
        const uptime = formatDuration(b.uptime_s);
        return `<div class="card glass board ${status}">
          <div class="row">
            <div class="col">
              <h4>${b.name||b.board_id} <small>${b.kind||''}</small></h4>
              <div class="dim">${b.board_id}</div>
            </div>
            <div class="col right">
              ${wifi}
              <div class="pill ${status}" title="${statusText}">${statusText.toUpperCase()}</div>
            </div>
          </div>
          <div class="kv slim">
            <div><span>IP</span><b>${b.last_ip||'—'}</b></div>
            <div><span>FW</span><b>${b.fw||'—'}</b></div>
            <div><span>MAC</span><b>${b.mac||'—'}</b></div>
            <div><span>Uptime</span><b>${uptime}</b></div>
            <div><span>WS URL</span><b class="mono">${b.ws_url||'—'}</b></div>
            <div><span>Token</span><b class="mono">${b.token||'—'}</b></div>
            <div><span>Último visto</span><b>${last}</b></div>
          </div>
        </div>`;
      }).join('');
    }catch(e){ toast('Error leyendo equipos','err', e.trace); }
  },
  async add(){
    const id = ($('#newBoardId').value||'').trim();
    const tok = ($('#newBoardToken').value||'').trim();
    const nm = ($('#newBoardName').value||'').trim();
    const kd = ($('#newBoardKind').value||'').trim();
    if(!id||!tok) return toast('ID y Token son obligatorios','err');
    try{
      await API.post('/api/boards', {board_id:id, token:tok, name:nm||undefined, kind:kd||undefined});
      toast('Equipo agregado');
      $('#newBoardId').value='';
      $('#newBoardToken').value='';
      $('#newBoardName').value='';
      $('#newBoardKind').value='';
      await Boards.refresh();
    }catch(e){ toast('No se pudo agregar','err', e.trace); }
  }
};

const Events={async refresh(){try{const r=await API.get('/api/events/tail?n=200'); $('#eventsTail').textContent=(r.lines||[]).join('\n');}catch(e){}}};
const Alerts={
  lastState:null,
  pumpOnSince:null,
  pumpStartPct:null,
  filterAlerted:false,
  lastSpokenAt:new Map(),
  audioCtx:null,
  init(){
    const resume=()=>this.resumeAudio();
    ['click','touchstart','keydown'].forEach(ev=>document.addEventListener(ev,resume,{once:true}));
  },
  resumeAudio(){
    const Ctx=window.AudioContext||window.webkitAudioContext;
    if(!Ctx) return;
    if(!this.audioCtx){
      try{this.audioCtx=new Ctx();}
      catch(_){this.audioCtx=null;return;}
    }
    if(this.audioCtx && this.audioCtx.state==='suspended'){
      this.audioCtx.resume().catch(()=>{});
    }
  },
  ensureCtx(){
    this.resumeAudio();
    return this.audioCtx||null;
  },
  playTone(freq=880,duration=0.25,volume=0.18,type='sine'){
    const ctx=this.ensureCtx();
    if(!ctx) return;
    try{
      const osc=ctx.createOscillator();
      const gain=ctx.createGain();
      osc.type=type;
      osc.frequency.value=freq;
      gain.gain.value=volume;
      osc.connect(gain).connect(ctx.destination);
      const now=ctx.currentTime;
      osc.start(now);
      osc.stop(now+Math.max(0.05,duration));
    }catch(_){/* ignore audio errors */}
  },
  speak(text,{lang='es-MX',rate=1,pitch=1}={}){
    if(!('speechSynthesis' in window)) return;
    try{
      const u=new SpeechSynthesisUtterance(text);
      u.lang=lang;
      u.rate=rate;
      u.pitch=pitch;
      window.speechSynthesis.speak(u);
    }catch(_){/* ignore speech errors */}
  },
  notify(text,{alert=false,throttleMs=7000}={}){
    const now=Date.now();
    const last=this.lastSpokenAt.get(text)||0;
    if(now-last<throttleMs) return;
    this.lastSpokenAt.set(text,now);
    if(alert){
      this.playTone(760,0.4,0.25,'square');
      const self=this;
      setTimeout(()=>self.playTone(520,0.6,0.22,'sawtooth'),450);
    }else{
      this.playTone(880,0.22,0.18,'sine');
    }
    this.speak(text,alert?{lang:'es-MX',rate:0.95,pitch:0.9}:{lang:'es-MX'});
  },
  cloneState(state){
    try{return structuredClone(state);}catch(_){return JSON.parse(JSON.stringify(state||{}));}
  },
  process(state,cfg){
    if(!state) return;
    if(!this.lastState){
      this.lastState=this.cloneState(state);
      return;
    }
    const prev=this.lastState;
    const now=Date.now();
    if(state.mode!==prev.mode){
      if(state.mode==='AUTO'){
        if(prev.mode==='MANUAL_OFF') this.notify('Finalización modo manual',{throttleMs:5000});
        this.notify('Inicio modo automático');
      }else if(state.mode==='MANUAL_OFF' && prev.mode!=='MANUAL_OFF'){
        this.notify('Inicio modo manual');
      }
    }
    const pumpOn=!!(state.actuator&&state.actuator.pump_on);
    const pumpPrev=!!(prev.actuator&&prev.actuator.pump_on);
    if(pumpOn!==pumpPrev){
      if(pumpOn){
        this.notify('Inicio de llenado de tolva');
        this.pumpOnSince=now;
        this.pumpStartPct=state.hopper?.filtered_pct??0;
        this.filterAlerted=false;
      }else{
        this.pumpOnSince=null;
        this.pumpStartPct=null;
        this.filterAlerted=false;
      }
    }
    const handleSensorStatus=(curr,prevStatus,label)=>{
      const currStat=(curr?.status||'').toUpperCase();
      const prevStat=(prevStatus?.status||'').toUpperCase();
      const currReason=curr?.status_reason||'';
      const prevReason=prevStatus?.status_reason||'';
      if(currStat!==prevStat){
        if(currStat==='FAULT'){
          const msg=`Falla sensor ${label}${currReason?': '+currReason:''}`;
          this.notify(msg,{alert:true,throttleMs:60000});
        }else if(currStat==='WARNING'){
          const msg=`Advertencia sensor ${label}${currReason?': '+currReason:''}`;
          this.notify(msg,{alert:true,throttleMs:35000});
        }else if(currStat==='OK' && (prevStat==='FAULT'||prevStat==='WARNING')){
          this.notify(`Sensor ${label} recuperado`,{throttleMs:15000});
        }
      }else if(currStat==='WARNING' && currReason && currReason!==prevReason){
        const msg=`Advertencia sensor ${label}${currReason?': '+currReason:''}`;
        this.notify(msg,{alert:true,throttleMs:35000});
      }
    };
    handleSensorStatus(state.tank, prev.tank, 'del tanque');
    handleSensorStatus(state.hopper, prev.hopper, 'de la tolva');
    const target=((cfg?.hopper?.cal?.target_pct??65)+(cfg?.auto?.anti_spill_margin_pct??1.2));
    const hopperPct=state.hopper?.filtered_pct??0;
    const hopperPrev=prev.hopper?.filtered_pct??0;
    if(hopperPct>=target && hopperPrev<target){
      this.notify('Tolva llena');
    }
    if(pumpOn){
      if(this.pumpOnSince===null){
        this.pumpOnSince=now;
        this.pumpStartPct=hopperPct;
        this.filterAlerted=false;
      }
      const runMs=now-(this.pumpOnSince||now);
      const rise=hopperPct-(this.pumpStartPct??hopperPct);
      const learnedMs=(state.auto_model?.ema_run_s??0)*1000;
      const limit=Math.max(learnedMs*1.8,180000);
      const minRise=Math.max(1.0,(cfg?.hopper?.cal?.hysteresis_pct??5)*0.2);
      if(!this.filterAlerted && runMs>limit && rise<minRise){
        this.notify('Atención. Se superó el tiempo de llenado y el nivel no aumentó. Limpiar el filtro.',{alert:true,throttleMs:60000});
        this.filterAlerted=true;
      }
    }
    this.lastState=this.cloneState(state);
  }
};
function bind(){
  const themeBtn=$('#themeToggle');
  themeBtn.addEventListener('click',()=>document.getElementById('app').classList.toggle('theme-light'));
  $$('.swatch').forEach(b=>b.addEventListener('click',()=>document.documentElement.style.setProperty('--accent', b.dataset.accent)));
  $('#apiKey').value=API.key;
  $('#saveKey').addEventListener('click',()=>{API.key=$('#apiKey').value.trim(); localStorage.setItem('apiKey',API.key); toast('API Key guardada')});

  Alerts.init();

  const headerEl=$('#app > header');
  const controlsToggle=$('#controlsToggle');
  const controlsLabel=controlsToggle?.querySelector('.label');
  const controlsChevron=controlsToggle?.querySelector('.chevron');
  const setCollapsed=(collapsed,{scroll=true}={})=>{
    if(!headerEl) return;
    headerEl.classList.toggle('collapsed',collapsed);
    headerEl.classList.toggle('expanded',!collapsed);
    if(controlsToggle){
      controlsToggle.setAttribute('aria-expanded',collapsed?'false':'true');
      if(controlsLabel) controlsLabel.textContent=collapsed?'Mostrar panel':'Ocultar panel';
      if(controlsChevron) controlsChevron.textContent=collapsed?'▾':'▴';
    }
    if(!collapsed && scroll){
      requestAnimationFrame(()=>headerEl.scrollIntoView({behavior:'smooth',block:'start'}));
    }
  };
  if(controlsToggle && headerEl){
    controlsToggle.addEventListener('click',()=>{
      const collapsed=!headerEl.classList.contains('collapsed');
      setCollapsed(collapsed);
    });
    const mq=window.matchMedia('(max-width:720px)');
    const applyResponsive=()=>setCollapsed(mq.matches,{scroll:false});
    applyResponsive();
    mq.addEventListener('change',applyResponsive);
  }

  const nav=$('#app nav.tabs');
  const tabToggle=$('#tabToggle');
  const tabLabel=$('#tabToggleLabel');
  const tabButtons=$$('#app nav.tabs .tab-menu button');
  const closeMenu=()=>{if(nav) nav.classList.remove('open'); if(tabToggle) tabToggle.setAttribute('aria-expanded','false');};
  const activateTab=async btn=>{
    tabButtons.forEach(x=>x.classList.remove('active'));
    $$('.tab').forEach(x=>x.classList.remove('active'));
    btn.classList.add('active');
    const id=btn.dataset.tab;
    if(tabLabel) tabLabel.textContent=btn.textContent.trim();
    $('#'+id).classList.add('active');
    if(id==='calib') await Calibration.refresh();
    if(id==='history') await History.refresh();
    if(id==='events') await Events.refresh();
    if(id==='boards') await Boards.refresh();
  };
  if(tabToggle){
    tabToggle.addEventListener('click',ev=>{
      ev.preventDefault();
      if(!nav) return;
      const isOpen=nav.classList.toggle('open');
      tabToggle.setAttribute('aria-expanded',isOpen?'true':'false');
    });
  }
  if(tabButtons.length){
    const active=tabButtons.find(b=>b.classList.contains('active'))||tabButtons[0];
    if(active && tabLabel) tabLabel.textContent=active.textContent.trim();
    tabButtons.forEach(btn=>btn.addEventListener('click',async()=>{closeMenu(); await activateTab(btn);}));
  }
  document.addEventListener('click',ev=>{if(nav && nav.classList.contains('open') && !nav.contains(ev.target)) closeMenu();});
  document.addEventListener('keydown',ev=>{if(ev.key==='Escape') closeMenu();});

  $('#btnManual').addEventListener('click',()=>Manual.toggle());
  $('#applyDuty').addEventListener('click',()=>Manual.applyDuty());
  $('#onInput').addEventListener('input',()=>$('#onVal').textContent=$('#onInput').value);
  $('#offInput').addEventListener('input',()=>$('#offVal').textContent=$('#offInput').value);
  $('#btnAuto').addEventListener('click',()=>Auto.toggle());
  $('#applyAuto').addEventListener('click',()=>Auto.apply());
  const tankSensorBtn=$('#tankSensorToggle');
  if(tankSensorBtn) tankSensorBtn.addEventListener('click',()=>Auto.toggleTankSensor());
  $('#btnSaveProf').addEventListener('click',()=>Profiles.save());
  $('#refreshHistory').addEventListener('click',()=>History.refresh());
  $('#btnSaveCal').addEventListener('click',()=>Calibration.save());
  $('#btnPatchCfg').addEventListener('click',async()=>{const origins=$('#origins').value.split(',').map(s=>s.trim()).filter(Boolean); const api_key=$('#apiKeyCfg').value.trim(); const payload={}; if(origins.length) payload.allowed_origins=origins; if(api_key.length) payload.api_key=api_key; try{await API.patch('/api/config',payload); toast('Config guardada')}catch(e){toast('Error guardando config','err',e.trace)}});
  $('#btnRefreshBoards').addEventListener('click',()=>Boards.refresh());
  $('#btnAddBoard').addEventListener('click',()=>Boards.add());
  const dashAutoBtn=$('#dashAutoToggle');
  const dashManualBtn=$('#dashManualToggle');
  if(dashAutoBtn) dashAutoBtn.addEventListener('click',()=>Auto.toggle());
  if(dashManualBtn) dashManualBtn.addEventListener('click',()=>Manual.toggle());
}
const History={data:[],async refresh(){try{const r=await API.get('/api/history?n=600'); this.data=r.items||[]; this.draw();}catch(e){toast('No se pudo leer histórico','err',e.trace)}}, draw(){const items=this.data; drawPlot($('#plotLevels'), items.map(x=>x.ts), [{label:'Tank %',values:items.map(x=>x.tank_pct)},{label:'Hopper %',values:items.map(x=>x.hopper_pct)}],'%'); drawPlot($('#plotRuntime'), items.map(x=>x.ts), [{label:'Runtime (s)',values:items.map(x=>x.runtime_s)}],'s'); drawPlot($('#plotPulses'), items.map(x=>x.ts), [{label:'Pulsos',values:items.map(x=>x.pulses)}],''); drawPlot($('#plotHz'), items.map(x=>x.ts), [{label:'Hz',values:items.map(x=>x.hz)}],'Hz'); drawPlot($('#plotDuty'), items.map(x=>x.ts), [{label:'Duty %',values:items.map(x=>x.duty)}],'%'); drawPlot($('#plotOnMs'), items.map(x=>x.ts), [{label:'ON ms',values:items.map(x=>x.on_ms)},{label:'OFF ms',values:items.map(x=>x.off_ms)}],'ms');}};
function drawPlot(c,xs,series,unit){const ctx=c.getContext('2d'),W=c.width,H=c.height; ctx.clearRect(0,0,W,H); ctx.fillStyle='#0a0f14'; ctx.fillRect(0,0,W,H); const padL=48,padR=10,padT=12,padB=22; ctx.strokeStyle='#203042'; ctx.beginPath(); ctx.moveTo(padL,H-padB); ctx.lineTo(W-padR,H-padB); ctx.moveTo(padL,padT); ctx.lineTo(padL,H-padB); ctx.stroke(); if(xs.length<2) return; let min=+Infinity,max=-Infinity; series.forEach(s=>s.values.forEach(v=>{ if(v<min)min=v; if(v>max)max=v; })); if(!isFinite(min)||!isFinite(max)) return; if(Math.abs(max-min)<1e-6){ max+=1; min-=1;} const x0=xs[0],x1=xs[xs.length-1]; const X=i=>padL+(W-padL-padR)*((xs[i]-x0)/(x1-x0)); const Y=v=>H-padB-(H-padT-padB)*((v-min)/(max-min)); const colors=[getComputedStyle(document.documentElement).getPropertyValue('--accent').trim()||'#00b3ff','#2bd97c','#ff7a59']; series.forEach((s,i)=>{ctx.strokeStyle=colors[i%colors.length]; ctx.lineWidth=2; ctx.beginPath(); for(let k=0;k<xs.length;k++){const x=X(k),y=Y(s.values[k]); if(k===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);} ctx.stroke();}); ctx.fillStyle='#9ab'; ctx.font='12px system-ui'; ctx.fillText(unit,W-padR-24,padT+12);}
async function tick(){ try{await API.get('/api/diag');}catch(e){toast('Servidor/Key no válidos. Revisa API Key (DEVKEY123).','err')} await Dash.refresh(); await Boards.refresh(); await Events.refresh(); setTimeout(tick,2000); }
bind(); Dash.refresh(); Profiles.refresh(); History.refresh(); Boards.refresh(); Events.refresh(); Calibration.refresh(); tick();
