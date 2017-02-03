var scene, camera, renderer;

var WIDTH  = window.innerWidth;
var HEIGHT = window.innerHeight;

var SPEED = 0.01;

function init() {
    scene = new THREE.Scene();

    initCube();
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

function initCube() {
    //var loader = new THREE.SceneLoader();
    var loader = new THREE.ObjectLoader();
    loader.load('./scene.json', function(result) {
        //scene = new THREE.Scene(geometry);
        //scene.add(mesh);
        scene = result;
        //alert(scene);
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
    renderer.render(scene,scene.getChildByName("Camera"));

}

init();
render();

