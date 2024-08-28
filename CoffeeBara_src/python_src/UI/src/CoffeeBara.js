const capIcon = document.querySelector('.cap_icon');
const holderIcon = document.querySelector('.holder_icon');
const capIconMoved = document.querySelector('.cap_icon_moved');
const holderIconMoved = document.querySelector('.holder_icon_moved');
const cupIcon = document.querySelector('.cup_icon');
const cup2Icon = document.querySelector('.cup2_icon');
const recycleIcon = document.querySelector('.recycle_icon');
const recycle1Icon = document.querySelector('.recycle1_icon');
const recycle2Icon = document.querySelector('.recycle2_icon');
const recycle3Icon = document.querySelector('.recycle3_icon');
const showerHeadIcon = document.querySelector('.shower_head_icon');
const wash1Icon = document.querySelector('.wash1_icon');
const wash2Icon = document.querySelector('.wash2_icon');
const defaultIcon = document.querySelector('.default_img');


const stageText = document.querySelector('.state_text');


const stateIcon = document.querySelector('.state_icon');
const cupInfoText = document.querySelector('.cup_info_text');

const sizeOfCurrent = document.querySelector('.size_of_current');
const holderOfCurrent = document.querySelector('.holder_of_current');
const stateOfCurrent = document.querySelector('.stage_of_current');

const sizeOfNext = document.querySelector('.size_of_next');
const holderOfNext = document.querySelector('.holder_of_next');
const stateOfNext = document.querySelector('.stage_of_next');

function getCalender(){
    var week = new Array('일요일', '월요일', '화요일', '수요일', '목요일', '금요일', '토요일');

    var now = new Date();
    var year  = now.getFullYear();
    var month = now.getMonth() + 1;
    var date  = now.getDate();
    
    var day = now.getDay();
    day = week[day];
    
    if(month < 10){
        month = '0' + month;
    }

    if(date < 10){
        date = '0' + date;
    }

    Today_date = year + month + date;
    
    calender.innerText = `${year}.${month}.${date} /  ${day}`;
}
// 시간 생성
function getClock(){
    var now   = new Date();
    var hours   = now.getHours();
    var minutes = now.getMinutes();
    var second = now.getSeconds();
    
    if(hours < 10){
        hours = '0' + hours;
    }
    if(minutes < 10){
        minutes = '0' + minutes;
    }
    if(second < 10){
        second = '0' + second;
    }
    
    if(hours >= 12){
        $('#daynight').text("오후");
    }else{
        $('#daynight').text("오전");
    }

    clock.innerText = `${hours}:${minutes}:${second}`;
}

getClock();
getCalender();
setInterval(getClock, 1000);
setInterval(getCalender, 1000);

// var socket = io('http://localhost:5113');

// var CurrentCUP = {
//     Holder : 0,
//     Size : 0
// };

// var NextCUP = {
//     Holder : 0,
//     Size : 0
// };

// function NextCUP_update(holder_exist, cup_size){
//     NextCUP.Holder = holder_exist;
//     NextCUP.Size = cup_size;
// }

// function CurrentCUP_update(){
//     CurrentCUP.Holder = NextCUP.Holder;
//     CurrentCUP.Size = NextCUP.Size;
// }

// // 서버로부터 'state update' 이벤트 수신
// socket.on('state update', function(data) {
//     console.log(data);
//     var main_state = data.state;
//     if(main_state === 1){
//         stage1();
//     }
//     else if(main_state === 2){
//         stage2();
//     }
//     else if(main_state === 3){
//         stage3();
//     }
// });

// // 서버로부터 'cup update' 이벤트 수신
// socket.on('cup update', function(data) {
//     var cup_info = data.cup;
    
//     cup_info = cup_info >> 5;
//     var cup_size = cup_info & 0x03;

//     cup_info = cup_info >> 2;
//     var holder_exist = cup_info & 0x01;

//     console.log(holder_exist, cup_size);
    
//     CurrentCUP_update();
//     NextCUP_update(holder_exist, cup_size);

//     if(!(NextCUP.Size)){
//         disappearNextCup();
//     }else{
//         getNextCup();

//         if(NextCUP.Size === 1){ sizeOfNext.textContent = "SIZE : Small"; }
//         else if(NextCUP.Size === 2){ sizeOfNext.textContent = "SIZE : Regular"; } 
//         else if(NextCUP.Size === 3){ sizeOfNext.textContent = "SIZE : Large"; } 

//         if(NextCUP.Holder === 0){ holderOfNext.textContent = "HOLDER : No"; }
//         else { holderOfNext.textContent = "HOLDER : Yes"; }
//     }

//     if(!(CurrentCUP.Size)){
//         sizeOfCurrent.textContent = "There's no cup";
//     }else{
//         if(CurrentCUP.Size === 1){ sizeOfCurrent.textContent = "SIZE : Small"; }
//         else if(CurrentCUP.Size === 2){ sizeOfCurrent.textContent = "SIZE : Regular"; } 
//         else if(CurrentCUP.Size === 3){ sizeOfCurrent.textContent = "SIZE : Large"; } 

//         if(CurrentCUP.Holder === 0){ holderOfCurrent.textContent = "HOLDER : No"; }
//         else { holderOfCurrent.textContent = "HOLDER : Yes"; }
//     }

// });

// socket.on('next cup', function(data){

//     if(data === 0){
//         disappearNextCup();
//     }
//     else{
//         getNextCup();

//         cup_info = data.cup;
    
//         cup_info = cup_info >> 5;
        
//         cup_size = cup_info & 0x03;

//         cup_info = cup_info >> 2;
//         holder_exist = cup_info & 0x01;

//         console.log(holder_exist, cup_size);

//         if(cup_size === 1){
//             sizeOfCurrent.textContent = "SIZE : Small";
//         }
//         else if(cup_size === 2){
//             sizeOfCurrent.textContent = "SIZE : Regular";
//         } 
//         else if(cup_size === 3){
//             sizeOfCurrent.textContent = "SIZE : Large";
//         } 

//         if(holder_exist === 0){
//             holderOfCurrent.textContent = "HOLDER : No";
//         }
//         else {
//             holderOfCurrent.textContent = "HOLDER : Yes";
//         }
//     }

// });

// // WebSocket 연결 확인
// socket.on('connect', function() {
//     console.log('WebSocket connected.');
// });

// // WebSocket 연결 끊김 확인
// socket.on('disconnect', function() {
//     console.log('WebSocket disconnected.');
// });
// var current_stage = 0;

document.addEventListener('DOMContentLoaded', function () {
    setTimeout(()=>{
        stage1();
    }, 5000);
    setTimeout(()=>{
        stage2();
    }, 8000);
    setTimeout(()=>{
        stage3();
    }, 14000);

    setTimeout(()=>{
        next_cup();
    }, 7000)
});

function stage1(){
    startAnimationStage1();
}

function startAnimationStage1() {
    defaultIcon.classList.add('hidden');
    capIcon.classList.remove('hidden');
    holderIcon.classList.remove('hidden');
    cup2Icon.classList.remove('hidden');

    stageText.textContent = '뚜껑, 홀더 분리 단계';
    stateOfCurrent.textContent = 'STAGE \n\n컵 분해 단계(#1)';

    capIcon.classList.add('moveUp');
    holderIcon.classList.add('moveDown');

    capIcon.addEventListener('animationend', handleMoveEnd, { once: true });
    holderIcon.addEventListener('animationend', handleMoveEnd, { once: true });
}


function handleBlinkEnd(event) {
    const element = event.target;
    element.classList.remove('blink');
    element.classList.add('hidden');

    cupIcon.classList.remove('hidden');
    cup2Icon.classList.add('hidden');
    stageText.textContent = '뚜껑, 홀더 분리 완료';
}


function handleMoveEnd(event) {
    const element = event.target;
    if (element.classList.contains('moveUp')) {
        element.classList.remove('moveUp');
    } else if (element.classList.contains('moveDown')) {
        element.classList.remove('moveDown');
    }

    element.classList.add('hidden');
    capIconMoved.classList.remove('hidden');
    holderIconMoved.classList.remove('hidden');
    
    setTimeout(() => {
        capIconMoved.classList.add('blink');
        holderIconMoved.classList.add('blink');
    }, 300); // 잠시 멈추고 깜빡임 시작

    capIconMoved.addEventListener('animationend', handleBlinkEnd, { once: true });
    holderIconMoved.addEventListener('animationend', handleBlinkEnd, { once: true });
}


function stage2(){
    startAnimationStage2();
}

function startAnimationStage2(){
    defaultIcon.classList.add('hidden');
    cupIcon.classList.add('moveDownCup');

    cupIcon.addEventListener('animationend', () => {
        showerHeadIcon.classList.remove('hidden');

        stageText.textContent = '컵 세척 중';
        stateOfCurrent.textContent = 'STAGE \n\n컵 세척 단계(#2)';

        setTimeout(() => {
            wash1Icon.classList.remove('hidden');
            wash2Icon.classList.remove('hidden');
        }, 300)
    }, { once: true });
}

function stage3(){
    startAnimationStage3();
}

function startAnimationStage3(){
    defaultIcon.classList.add('hidden');
    cupIcon.classList.add('hidden');
    showerHeadIcon.classList.add('hidden');
    wash1Icon.classList.add('hidden');
    wash2Icon.classList.add('hidden');

    recycle1Icon.classList.remove('hidden');
    recycle2Icon.classList.remove('hidden');
    recycle3Icon.classList.remove('hidden');
    // recycleIcon.classList.remove('hidden');
    // recycleIcon.classList.add('rotate');

    stageText.textContent = '컵 재활용 분류 중';
    stateOfCurrent.textContent = 'STAGE \n\n컵 분류 단계(#3)';

    setTimeout(() => {
        defaultIcon.classList.remove('hidden');
        recycle1Icon.classList.add('hidden');
        recycle2Icon.classList.add('hidden');
        recycle3Icon.classList.add('hidden');
        stageText.textContent = '먹고 남은 커피컵 저한테 주세요';
        stateOfCurrent.textContent = 'STAGE ';
        sizeOfCurrent.textContent = 'SIZE ';
        holderOfCurrent.textContent = 'HOLDER ';
    }, 5000)
}

const currentCupDiv = document.querySelector(".current_cup");
const nextCupDiv = document.querySelector(".next_cup");
const iconDiv = document.querySelector(".state_icon_div");

function next_cup(){
    const currentCupDiv = document.querySelector(".current_cup");
    const nextCupDiv = document.querySelector(".next_cup");
    const iconDiv = document.querySelector(".state_icon_div");

    // 서버에서 값을 받아오는 함수 (예제용으로 랜덤 값을 생성)
    const fetchCupInfos = () => {
        // 이 예제에서는 0, 1, 2를 랜덤하게 반환
        const cupInfos = Math.floor(Math.random() * 3);
        return cupInfos;
    };

    // 애니메이션 및 레이아웃 조정 함수
    const updateLayout = (cupInfos) => {
        if (cupInfos <= 1) {
            // next_cup이 오른쪽으로 사라지는 애니메이션
            nextCupDiv.classList.add("slide-out-right");
            nextCupDiv.classList.remove("slide-in-right");
            // current_cup이 가운데 정렬되고 너비가 확장
            currentCupDiv.classList.add("align-cup-div-center");
            currentCupDiv.classList.remove("align-cup-div-left");
            // icon_div가 가운데 정렬되고 너비가 확장
            iconDiv.classList.add("align-icon-div-center");
            iconDiv.classList.remove("align-icon-div-left");
        } else {
            // next_cup이 오른쪽에서 나오는 애니메이션
            nextCupDiv.classList.remove("slide-out-right");
            nextCupDiv.classList.add("slide-in-right");
            // current_cup이 다시 원래 너비로 축소
            currentCupDiv.classList.remove("align-cup-div-center");
            currentCupDiv.classList.add("align-cup-div-left");
            // iconDiv가 다시 원래 너비로 축소
            iconDiv.classList.remove("align-icon-div-center");
            iconDiv.classList.add("align-icon-div-left");
        }
    };

    setInterval(() => {
        const cupInfos = fetchCupInfos();
        updateLayout(cupInfos);
    }, 2000); // 1초마다 실행
}


function getNextCup(){
    // next_cup이 오른쪽에서 나오는 애니메이션
    nextCupDiv.classList.remove("slide-out-right");
    nextCupDiv.classList.add("slide-in-right");
    // current_cup이 다시 원래 너비로 축소
    currentCupDiv.classList.remove("align-cup-div-center");
    currentCupDiv.classList.add("align-cup-div-left");
    // iconDiv가 다시 원래 너비로 축소
    iconDiv.classList.remove("align-icon-div-center");
    iconDiv.classList.add("align-icon-div-left");
}

function disappearNextCup(){
    // next_cup이 오른쪽으로 사라지는 애니메이션
    nextCupDiv.classList.add("slide-out-right");
    nextCupDiv.classList.remove("slide-in-right");
    // current_cup이 가운데 정렬되고 너비가 확장
    currentCupDiv.classList.add("align-cup-div-center");
    currentCupDiv.classList.remove("align-cup-div-left");
    // icon_div가 가운데 정렬되고 너비가 확장
    iconDiv.classList.add("align-icon-div-center");
    iconDiv.classList.remove("align-icon-div-left");
}