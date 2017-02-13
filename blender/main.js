var scene, camera, renderer, controls;

var WIDTH  = window.innerWidth;
var HEIGHT = window.innerHeight;

var SPEED = 0.01;

function init() {
    scene = new THREE.Scene();

    initScene();
    initCamera();
    initRenderer();

    document.body.appendChild(renderer.domElement);
    

}

function initCamera() {
    camera = new THREE.PerspectiveCamera(70, WIDTH / HEIGHT, 1, 10);
    camera.position.set(0, 3.5, 5);
    camera.lookAt(scene.position);
}

function initRenderer() {
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(WIDTH, HEIGHT);
}

function initScene() {
    //var loader = new THREE.SceneLoader();
    var loader = new THREE.ObjectLoader();
    loader.load('./scene.json', function(result) {
        //scene = new THREE.Scene(geometry);
        //scene.add(mesh);
        scene = result;
        //alert(scene);

        controls = new THREE.TrackballControls(scene.getObjectByName("Camera"));
        //controls.target.set( 0, 0, 0 )

				controls.rotateSpeed = 1.0;
				controls.zoomSpeed = 1.2;
				controls.panSpeed = 0.8;

				controls.noZoom = false;
				controls.noPan = false;

				controls.staticMoving = true;
				controls.dynamicDampingFactor = 0.3;

				controls.keys = [ 65, 83, 68 ];

				controls.addEventListener( 'change', render );

        animate();
        render();
        //alert("boo");
    });
    
    //cube = new THREE.Mesh(new THREE.CubeGeometry(2, 2, 2), new THREE.MeshNormalMaterial());
    //scene.add(cube);
}

function rotateCube() {
    cube.rotation.x -= SPEED * 2;
    cube.rotation.y -= SPEED;
    cube.rotation.z -= SPEED * 3;
}

function render() {
    //requestAnimationFrame(render);
    //rotateCube();
    //renderer.render(scene, camera);
    renderer.render(scene,scene.getObjectByName("Camera"));

}

function start_socket() {
    var socket = io.connect('http://' + document.domain + ':' + location.port + '/');
    
    socket.on('posupdate', function(msg) {
        var x = msg.x;
        var y = msg.y;
        var z = msg.z;
        var rx = msg.rx;
        var ry = msg.ry;
        var rz = msg.rz;
        console.log("update got x:" + x + " and y:" + y + " and z: " + z + " rx:" + rx + " and ry:" + ry + " and rz: " + rz);
        scene.getObjectByName("VW Coccinelle").position.x = x;
        scene.getObjectByName("VW Coccinelle").position.y = z;
        scene.getObjectByName("VW Coccinelle").position.z = y;
        scene.getObjectByName("VW Coccinelle").rotation.x = rx;
        scene.getObjectByName("VW Coccinelle").rotation.y = ry;
        scene.getObjectByName("VW Coccinelle").rotation.z = rz;
        render();
    });
    //alert('http://' + document.domain + ':' + location.port + '/pos');

}



function animate() {
        requestAnimationFrame( animate );
        controls.update();
}

$(document).ready(function(){
    start_socket();
})

init();
//render();

