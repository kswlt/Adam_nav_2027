// Operator confirmation is explicit: this module never auto-publishes a map.
const reviewSection = document.createElement('section');
reviewSection.innerHTML = `<h2>2a · 俯视检查与可通行区域</h2>
<div class="row"><label>分辨率(m) <input style="width:75px;min-width:0" id="review_resolution" type="number" value="0.05" step="0.01"></label><label>障碍最低z <input style="width:75px;min-width:0" id="review_min_z" type="number" value="-0.10" step="0.05"></label><label>最高z <input style="width:75px;min-width:0" id="review_max_z" type="number" value="1.50" step="0.05"></label><button class="action" id="prepare_review">生成所选地图检查图</button></div>
<p class="muted">先在上方选择已保存的原生地图。高度相对建图原点，不能直接当作离地高度；请结合地面检查。灰=未知，黑=高度范围内障碍，青线=雷达轨迹，绿=你圈选的可通行区域。空白不会自动变成自由空间。</p>
<div id="review_info" class="muted">尚未生成检查图</div><div style="max-height:600px;overflow:auto;background:#27313b;margin:10px 0"><canvas id="review_canvas" style="display:block;max-width:100%;height:auto;cursor:crosshair"></canvas></div>
<div class="row"><button id="finish_polygon">完成当前区域</button><button id="undo_vertex">撤销顶点</button><button id="clear_polygons">清空圈选</button><span id="polygon_count" class="muted"></span></div>
<p class="muted">在图上依次点击至少3个顶点，再点“完成当前区域”；可圈多个区域。障碍格会保留。发布前确认真实场地、地面高度和障碍物，不能只凭图上颜色判断。</p>
<div class="row"><input id="reviewer" placeholder="实际检查人的姓名/标识"><label><input style="min-width:0" type="checkbox" id="review_confirm">我已检查本次地图的可通行区域、地面高度和障碍物</label><button class="primary action" id="publish_review">确认并发布本地导航地图</button></div>
<h2 style="margin-top:18px">2b · 影子导航目标</h2><div class="row"><label>map x(m) <input style="width:85px;min-width:0" id="goal_x" type="number" value="0" step="0.1"></label><label>map y(m) <input style="width:85px;min-width:0" id="goal_y" type="number" value="0" step="0.1"></label><label>yaw(rad) <input style="width:85px;min-width:0" id="goal_yaw" type="number" value="0" step="0.1"></label><button class="action" id="submit_goal">发送影子目标</button><button class="danger action" id="cancel_task">取消当前任务</button></div><p class="muted">先停止建图、启动正式地图的影子导航并等待健康。x/y是地图绝对位置，不是相对前进距离；此入口不打开物理串口。</p>`;
el('logcomponent').closest('section').before(reviewSection);
let reviewData = null, reviewPolygons = [], currentPolygon = [], baseImage = null;
const canvas = el('review_canvas'), ctx = canvas.getContext('2d');
canvas.width = 1; canvas.height = 1;
function renderReview() {
  if (!reviewData) return;
  ctx.putImageData(baseImage, 0, 0);
  const draw = (vertices, closed) => {
    if (!vertices.length) return;
    ctx.beginPath(); ctx.moveTo(...vertices[0]); vertices.slice(1).forEach(v=>ctx.lineTo(...v));
    if (closed) {ctx.closePath();ctx.fillStyle='rgba(50,210,120,0.45)';ctx.fill();}
    ctx.strokeStyle=closed?'#1b9c48':'#d28b00';ctx.lineWidth=2;ctx.stroke();
    for (const [x,y] of vertices) {ctx.beginPath();ctx.arc(x,y,3,0,Math.PI*2);ctx.fillStyle='#b75d00';ctx.fill();}
  };
  reviewPolygons.forEach(v=>draw(v,true));draw(currentPolygon,false);
  // Re-draw occupied observations on top: polygons do not erase obstacles.
  const pixels=ctx.getImageData(0,0,canvas.width,canvas.height);
  for(let i=0;i<reviewData.raw.length;i++)if(reviewData.raw[i]===0){pixels.data[4*i]=12;pixels.data[4*i+1]=12;pixels.data[4*i+2]=12;}
  ctx.putImageData(pixels,0,0);
  ctx.strokeStyle='#00b8d9';ctx.lineWidth=1.5;ctx.beginPath();reviewData.path.forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y));ctx.stroke();
  el('polygon_count').textContent=`已完成${reviewPolygons.length}个区域，当前${currentPolygon.length}个顶点`;
}
async function loadReview() {
  try {
    const data=await api('/api/review');
    if(reviewData?.review_id===data.review_id)return;
    reviewData=data;reviewPolygons=[];currentPolygon=[];el('review_confirm').checked=false;
    data.raw=Uint8Array.from(atob(data.cells),c=>c.charCodeAt(0));canvas.width=data.width;canvas.height=data.height;
    baseImage=ctx.createImageData(data.width,data.height);
    data.raw.forEach((v,i)=>{const c=v===0?12:130;baseImage.data.set([c,c,c,255],4*i)});
    el('review_info').textContent=`${data.width}×${data.height}格，${data.resolution}m/格；原点(${data.origin[0].toFixed(2)},${data.origin[1].toFixed(2)})，左→右为+x，下→上为+y。${data.point_count}点/${data.keyframes}关键帧，障碍格${data.occupied_cells}；尚无已确认自由格。`;
    renderReview();
  } catch(error) {el('result').textContent=error.message;}
}
canvas.addEventListener('click',event=>{
  if(!reviewData)return;
  if(currentPolygon.length>=64){el('result').textContent='单个区域最多64个顶点';return;}
  const box=canvas.getBoundingClientRect();
  currentPolygon.push([Math.min(canvas.width-.001,Math.max(0,(event.clientX-box.left)*canvas.width/box.width)),Math.min(canvas.height-.001,Math.max(0,(event.clientY-box.top)*canvas.height/box.height))]);
  el('review_confirm').checked=false;renderReview();
});
el('finish_polygon').onclick=()=>{if(currentPolygon.length<3){el('result').textContent='至少选择3个顶点';return;}reviewPolygons.push(currentPolygon);currentPolygon=[];el('review_confirm').checked=false;renderReview();};
el('undo_vertex').onclick=()=>{currentPolygon.pop();el('review_confirm').checked=false;renderReview();};
el('clear_polygons').onclick=()=>{currentPolygon=[];reviewPolygons=[];el('review_confirm').checked=false;renderReview();};
el('prepare_review').onclick=()=>action('prepare_review',{native:el('native').value,resolution:Number(el('review_resolution').value),min_z:Number(el('review_min_z').value),max_z:Number(el('review_max_z').value)});
el('publish_review').onclick=()=>{
  if(!reviewData||currentPolygon.length||!reviewPolygons.length||!el('review_confirm').checked||!el('reviewer').value.trim()){el('result').textContent='先完成圈选、填写实际检查人并勾选检查确认';return;}
  action('publish_review',{review_id:reviewData.review_id,polygons:reviewPolygons,reviewer:el('reviewer').value,confirmed:true});
};
el('cancel_task').onclick=()=>action('cancel_task');
el('submit_goal').onclick=()=>action('goal',{x:Number(el('goal_x').value),y:Number(el('goal_y').value),yaw:Number(el('goal_yaw').value)});
let handledPublication='';
setInterval(()=>{
  if(statusCache?.review_id&&statusCache.review_id!==reviewData?.review_id)loadReview();
  const job=statusCache?.job;
  if(job?.action==='publish_review'&&job.ok&&job.result!==handledPublication){
    const candidate=job.result.split('：').pop();
    if(statusCache.maps.manifests.includes(candidate))el('manifest').value=candidate;
    handledPublication=job.result;
  }
},1000);
