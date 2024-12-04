const uint8_t index_ov2640_html[] = R"=====(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>+Admin Dashboard+</title>
<style>
@media only screen and (min-width: 850px) {
	body {
		 display: flex;
	}
	 #content-right {
		 margin-left: 10px;
	}  
}
body {
     font-family: Arial, Helvetica, sans-serif;
     background: #181818;
     color: #EFEFEF;
     font-size: 16px;
}
 #content-left {
     max-width: 400px;
	 flex: 1;
}
 #content-right {
     max-width: 400px;
	 flex: 1;
}
 #stream {
     width: 100%;
}
 .status-display {
     height: 25px;
     border: none;
     padding: 10px;
     font: 18px/22px sans-serif;
     margin-bottom: 10px;
     border-radius: 5px;
     background: green;
     text-align: center;
}
 #person {
     width:100%;
     height: 25px;
     border: none;
     padding: 20px 10px;
     font: 18px/22px sans-serif;
     margin-bottom: 10px;
     border-radius: 5px;
     resize: none;
     box-sizing: border-box;
}
 button {
     display: block;
     margin: 5px 0;
     padding: 0 12px;
     border: 0;
     width: 48%;
     line-height: 28px;
     cursor: pointer;
     color: #fff;
     background: #ff3034;
     border-radius: 5px;
     font-size: 16px;
     outline: 0;
}
 .buttons-mode button {
     background: #626ada;
}
 .buttons-mode button:hover {
     background: #4651e8;
}
 .buttons-mode button.active {
     background: #41d65f;
}
 .buttons-mode button.active:hover {
     background: #25d348;
}
 .buttons-mode button.active:disabled {
     cursor: default;
     background: #a0a0a0;
}
 .buttons {
     height:40px;
}
 #button-register {
     background: #f4a231;
}
 #button-register:hover {
     background: #f49511;
}
 #button-register:disabled {
     cursor: default;
     background: #a0a0a0;
}
 button:hover {
     background: #ff494d;
}
 button:active {
     background: #f21c21;
}
 button:disabled {
     cursor: default;
     background: #a0a0a0;
}
 .left {
     float: left;
}
 .right {
     float: right;
}
 .image-container {
     position: relative;
}
 .stream {
     max-width: 400px;
}
 ul {
     list-style: none;
     padding: 5px;
     margin:0;
}
 li {
     padding: 5px 0;
}
 .delete {
     background: #ff3034;
     border-radius: 5px;
     color: #fff;
     text-align: center;
     line-height: 18px;
     cursor: pointer;
     padding: 5px;
     margin-right: 20px;
}
 h3 {
     margin-bottom: 3px;
}
</style>
</head>
<body>
<div id="content-left">
  <div id="stream-container" class="image-container"> <img id="stream" src=""> </div>
</div>
<div id="content-right">
  <div class="status-display"> <span id="current-status"></span> </div>
  <h3>Mode</h3>
  <div class="buttons-mode">
    <button id="button-mode-register">REGISTER</button>
    <button id="button-mode-auto">AUTO LOCK</button>
    <button id="button-mode-manual" class="active">MANUAL LOCK</button>
  </div>
  <div id="door-control" style="margin-bottom: 70px;">
    <h3>Door Lock Control</h3>
    <div class="buttons-mode"><button id="button-open" class="left active">OPEN</button></div>
    <button id="button-close" class="right" disabled>CLOSE</button>
  </div>
  <div class="register">
    <h3>Register Face</h3>
    <div id="person-name">
      <input id="person" type="text" value="" placeholder="Masukin namanya disini" disabled>
    </div>
    <button id="button-register" title="Enter a name above before capturing a face" disabled>REGISTER</button>
  </div>
  <div class="people">
    <h3>Face Database</h3>
    <ul>
    </ul>
  </div>
  <div class="buttons">
    <button id="delete_all">DELETE ALL</button>
  </div>
</div>
<script>
document.addEventListener("DOMContentLoaded", function(event) {
  var baseHost = document.location.origin;
  var streamUrl = baseHost + ":81";
  const WS_URL = "ws://" + window.location.host + ":82";
  const ws = new WebSocket(WS_URL);

  const view = document.getElementById("stream");

  const buttonRegisterMode = document.getElementById("button-mode-register");
  const buttonAutoMode = document.getElementById("button-mode-auto");
  const buttonManualMode = document.getElementById("button-mode-manual");

  const nameField = document.getElementById("person");
  const registerButton = document.getElementById("button-register");
  const deleteAllButton = document.getElementById("delete_all");

  const openButton = document.getElementById("button-open");
  const closeButton = document.getElementById("button-close");

  ws.onopen = () => {
    console.log(`Connected to ${WS_URL}`);
  };
  ws.onmessage = message => {
    if (typeof message.data === "string") {
      if (message.data.substr(0, 8) == "listface") {
        addFaceToScreen(message.data.substr(9));
      } else if (message.data == "delete_faces") {
        deleteAllFacesFromScreen();
      } else {
        document.getElementById("current-status").innerHTML = message.data;
      }
    }
    if (message.data instanceof Blob) {
      var urlObject = URL.createObjectURL(message.data);
      view.src = urlObject;
    }
  };

  buttonRegisterMode.onclick = () => {
    if(!buttonRegisterMode.classList.contains("active")) {
      buttonRegisterMode.classList.add("active");
    }
    buttonAutoMode.classList.remove("active");
    buttonManualMode.classList.remove("active");

    nameField.value = "";
    nameField.disabled = false;
    registerButton.disabled = false;

    document.getElementById("door-control").style.visibility = "hidden";

    closeButton.click();
  };
  buttonAutoMode.onclick = () => {
    if(!buttonAutoMode.classList.contains("active")) {
      buttonAutoMode.classList.add("active");
    }
    buttonRegisterMode.classList.remove("active");
    buttonManualMode.classList.remove("active");

    document.getElementById("door-control").style.visibility = "hidden";

    nameField.disabled = true;
    registerButton.disabled = true;

    closeDoor();

    ws.send("auto_door_lock");
  };
  buttonManualMode.onclick = () => {
    if(!buttonManualMode.classList.contains("active")) {
      buttonManualMode.classList.add("active");
    }
    buttonAutoMode.classList.remove("active");
    buttonRegisterMode.classList.remove("active");

    document.getElementById("door-control").style.visibility = "visible";

    nameField.disabled = true;
    registerButton.disabled = true;

    closeDoor();

    ws.send("manual_door_lock");
  };

  registerButton.onclick = () => {
    person_name = nameField.value;
    if(person_name == "") alert("Enter a name above before registering a face");
    else ws.send("capture:" + person_name);
  };
  deleteAllButton.onclick = () => {
    ws.send("delete_all");
  };

  openButton.onclick = () => {
    openDoor();
    ws.send("open_door");
  }
  closeButton.onclick = () => {
    closeDoor();
    ws.send("close_door");
  }

  function deleteAllFacesFromScreen() {
    // deletes face list in browser only
    const faceList = document.querySelector("ul");
    while (faceList.firstChild) {
      faceList.firstChild.remove();
    }
    nameField.value = "";
    registerButton.disabled = true;
  }

  function addFaceToScreen(person_name) {
    const faceList = document.querySelector("ul");
    let listItem = document.createElement("li");
    let closeItem = document.createElement("span");
    closeItem.classList.add("delete");
    closeItem.id = person_name;
    closeItem.addEventListener("click", function() {
      ws.send("remove:" + person_name);
    });
    listItem.appendChild(closeItem).textContent = "X";
    listItem.appendChild(
      document.createElement("strong")
    ).textContent = person_name;
    faceList.appendChild(listItem);
    nameField.value = "";
  }

  function openDoor() {
    openButton.disabled = true;
    closeButton.disabled = false;
  }

  function closeDoor() {
    openButton.disabled = false;
    closeButton.disabled = true;
  }
});
</script>
</body>
</html>)=====";

size_t index_ov2640_html_len = sizeof(index_ov2640_html)-1;
