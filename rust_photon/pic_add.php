<!DOCTYPE html>
<html>

<!-- HTML COMMENT
//
// This is using fancybox ajax to provide a webpage with grouped pictures for each area (large and small)
//
// uses fancybox to install on raspbian os :-
// sudo apt install npm
// npm install fancybox --save-dev
// sudo apt install libjs-jquery
//
// implementation using fancybox
// https://www.npmjs.com/package/fancybox
// https://fancyapps.com/playground/GQ
// https://codepen.io/fancyapps/pen/zNWLoG
//
    HTML COMMENT      -->
<head>
    <meta charset="utf-8">
    <title>Pictures for Area B</title>

    <!-- CSS -->
    <link rel="stylesheet" type="text/css" href="jquery.fancybox.min.css">
    <script type="text/javascript" src="http://ajax.googleapis.com/ajax/libs/jquery/1.7/jquery.min.js"></script>
    <link rel="stylesheet" href="/fancybox/jquery.fancybox.css" type="text/css" media="screen" />
    <link rel="stylesheet" href="form_test.css" type="text/css" media="screen" />
    <script type="text/javascript" src="/fancybox/jquery.fancybox.pack.js"></script>
<style>
      .border {
        width: 200px;
        padding: 10px;
        margin: 20px;
      }
      .round_border {
        width: 200px;
        height: 120px;
        border:solid 3px #ff0000;
        border-radius: 15%;
        margin: 10px;
        padding: 6px;
      }
      .input-checkbox-item-cybozu input:checked:disabled+label::after {
         background-image: url('http://airobot.ddns.net:8029/dashboard/fancybox/2.jpg');
      }
      .multi_color_box {
        border-left: solid 3px red;
        border-top: solid 2px blue;
        border-right: solid 3px green;
        border-bottom: solid 4px orange;
      }
      .circle {
        border-radius: 50%;
      }
      .sample-border-box {
        -webkit-box-sizing: border-box;
        -moz-box-sizing: border-box;
        box-sizing: border-box;
        background: #ffcccc;
        border: 10px solid #993333;
        padding: 10px;
        width: 80px;
        height: 80px;
      }
      .sample-content-box {
        -webkit-box-sizing: content-box;
        -moz-box-sizing: content-box;
        box-sizing: content-box;
        background: #ffcccc;
        border: 10px solid #993333;
        padding: 10px;
        width: 80px;
        height: 80px;
      }
      .hr1 {
        border-width: 1px 0px 0px 0px;
        border-style: dashed;
        border-color: gray;
        height: 1px;
      }
      .title_bar1 {
        font-size: 24px;
        width: 500px;
        height: 36px;
        border-left:solid 8px #00ddff;
        border-bottom:solid 2px #00ddff;
        margin: 10px;
        padding: 6px;
      }
      .dot_dash {
        border-top: 9px dotted #f45e5e;
        border-bottom: 5px dashed #ffa5f0;
      }
      .border-solid {
        border: 3px solid #ff6347;
      }
      .border-double {
        border: 3px double #0000ff;
      }
      .border-groove {
        border: 10px groove #ffa500;
      }
      .border-ridge {
        border: 10px ridge #008000;
      }
      .border-inset {
        border: 10px inset #ffd700;
      }
      .border-outset {
        border: 10px outset #ff1493;
      }
      .border-dashed {
        border: 5px dashed #00bfff;
      }
      .border-dotted {
        border: 5px dotted #dc143c;
      }

.sub_img2{
    background:url('../lhs_upper.jpg');
    border: medium none;
    cursor: pointer;
    width: 850px;
    height: 505px;
}
.sub_img3{
    background:url('../rhs_upper.jpg');
    border: medium none;
    cursor: pointer;
    width: 850px;
    height: 505px;
}
.sub_img4{
    background:url('../lhs_lower.jpg');
    border: medium none;
    cursor: pointer;
    width: 850px;
    height: 505px;
}
.sub_img5{
    background:url('../rhs_lower.jpg');
    border: medium none;
    cursor: pointer;
    width: 850px;
    height: 505px;
}

.check-box {
  cursor: pointer;
}
.check-text {
  align-items: center;
  display: flex;
}
.check-box input {
  display: none;
}
.check-box input + .check-text::before {
  background-image: url("http://airobot.ddns.net:8029/dashboard/thumbnails/AbbeyRoof%20(105)_s_green.png");
  background-position: center;
  background-repeat: no-repeat;
  background-size: contain;
  content: "";
  height: 200px;
  position: relative;
  width: 200px;
}
.check-box input:checked + .check-text::before {
  background-image: url("http://airobot.ddns.net:8029/dashboard/thumbnails/AbbeyRoof%20(105)_s_red.png");
}
.check-box2 {
  cursor: pointer;
}
.check-box2 input {
  display: none;
}
.check-box2 input + .check-text::before {
  background-image: url("http://airobot.ddns.net:8029/dashboard/thumbnails/AbbeyRoof%20(105)_gr.png");
  background-position: center;
  background-repeat: no-repeat;
  background-size: contain;
  content: "";
  height: 200px;
  position: relative;
  width: 200px;
}
.check-box2 input:checked + .check-text::before {
  background-image: url("http://airobot.ddns.net:8029/dashboard/thumbnails/AbbeyRoof%20(105)_g.png");
}
.check-box3 {
  cursor: pointer;
}
.check-box3 input {
  display: none;
}
.check-box3 input + .check-text::before {
  background-image: url("http://airobot.ddns.net:8029/dashboard/thumbnails/AbbeyRoof%20(105)_dg.png");
  background-position: center;
  background-repeat: no-repeat;
  background-size: contain;
  content: "";
  height: 200px;
  position: relative;
  width: 200px;
}
.check-box3 input:checked + .check-text::before {
  background-image: url("http://airobot.ddns.net:8029/dashboard/thumbnails/AbbeyRoof%20(105)_s_red.png");
}
p {
  font-size: 24px;
}
/* mozilla web kit for pull down menu */
.select-test {
        -moz-appearance: none;
        -webkit-appearance: none;
        appearance: none;
        background: transparent url(https://haniwaman.com/cms/wp-content/uploads/2018/12/form-css-arrow.png) no-repeat center right 8px/16px 16px;
        border: 1px solid rgba(0, 0, 0, 0.16);
        border-radius: 0;
        color: inherit;
        cursor: pointer;
        font-family: inherit;
        font-size: 1em;
        padding: 0.4em 0.8em;
        width: 100%;
}

.select-test::-ms-expand {
        display: none;
}

.select-test:focus {
        border: 1px solid rgba(0, 0, 0, 0.32);
        box-shadow: none;
        outline: none;
}
  </style>
</head>

<script>
    $(document).ready(function() {
        $('.fancybox').fancybox({
            padding : 0,
            openEffect  : 'elastic'
        });
    });
</script>
<style>
img {
  border: 1px solid #ddd; /* Gray border */
  border-radius: 4px;  /* Rounded border */
  padding: 5px; /* Some padding */
  width: 150px; /* Set a small width */
}

/* Add a hover effect (blue shadow) */
img:hover {
  box-shadow: 0 0 2px 1px rgba(0, 140, 186, 0.5);
}
body {
  background-image:  url("bg01.jpg");
  background-size: cover;
}
</style>
<body >
<!-- HTML COMMENT
//
// This code would be created dynamically
// Query->"area" -> picture id(s)
// for each picture....
// Query->"picture id" -> picture url
//
    HTML COMMENT      -->

<p>
  <label for="check">
    <input type="checkbox" id="check">This is the standard check box that you have
  </label>
</p>
<p>
  <label class="check-box" for="check01">
    <input type="checkbox" id="check01">
    <span class="check-text">These are maybe better</span>
  </label>
</p>
<p>
  <label class="check-box2" for="check02">
    <input type="checkbox" id="check02">
    <span class="check-text">changes shape on selection</span>
  </label>
</p>
<p>
  <label class="check-box3" for="check03">
    <input type="checkbox" id="check03">
    <span class="check-text">chosen from the sytle sheet css</span>
  </label>
</p>
<br> Here is another example of a selection box (using mozilla tools) <br />
<br><br />
<select class="select-test">
        <option>-- Area A --</option>
        <option>-- Area B --</option>
        <option>-- Area C --</option>
        <option>-- Area D --</option>
</select>
<br> <br />
<a href="https://source.unsplash.com/7bwQXzbF6KE/1500x1000" class="fancybox" rel="gallery" title="Sample title 1"><img src="https://source.unsplash.com/7bwQXzbF6KE/240x160" /></a>
<a href="2.jpg" class="fancybox" rel="gallery" title="Sample title 1"><img src="2_s.jpg" /></a>

<table border="1" bgcolor="#CCDDFF"> <tr> <td bgcolor="red"> simple table </td> < > <td> </td> </tr>


<tr> <td> 2nd line </td> <td bgcolor="#8888ff"> 2nd line </td> </tr> </table>

<script language="JavaScript" type="text/javascript">
<!--
function AllChecked(){
 var check = document.form.aaa.checked;
 document.form.elements['bbb[0]'].checked = check;
 document.form.elements['bbb[1]'].checked = check;
 document.form.elements['bbb[2]'].checked = check;
 document.form.elements['bbb[3]'].checked = check;
 document.form.elements['bbb[4]'].checked = check;
}
//-->
</script>
<style>
.box-1 {
  width: 300px;
  background-color: skyblue;
  font-size: 50px;
}
</style>
<ul class="original_checkbox">
<div class="form-text-field-all">
<form action="../put_pic_in_area.php" name="form1" method="POST">
  <li>

<p>
   <center>
   <label for="restrict">Select an Area </label>
   <select id="restrict" name="areas[]" multiple size="3">
      <option value="Area A" selected>Area A</option>
      <option value="Area B" selected>Area B</option>
      <option value="Area C">Area C</option>
      <option value="Area D">Area D</option>
      <option value="Area E">Area E</option>
      <option value="Area F">Area F</option>
      <option value="Area G">Area G</option>
      <option value="Area H">Area H</option>
   </select>
</p>
  <div class="border dot_dash" >
  <input type="checkbox" name="aa1" onClick="AllCheck2();" /> SelectAll / Un-SelectAll
  </div>

   </center>
<table border="3" cellpadding="1" cellspacing="1" width="200" bordercolor="#000000">
<tr bgcolor="#e1f0ff">
<td align="center">
  <label class="check-box" for="check12">
    <input type="checkbox" name="bbb1[5]" value="../thumbnails/AbbeyRoof%20(105)_s.jpg" id="check12" checked="checked">
    <span class="check-text">changes <br /> border <br /> colour on selection <br />(No.6)</span>
  </label>
  </li>
</td>
<td align="center">
  <li>
    <center>
    <input type="checkbox" name="bbb1[0]" value="../thumbnails/AbbeyRoof (100)_s.jpg" id="cbox1" checked="checked"/>Picture 1
  </center>
    <label for="original_1">This time the border changes on checkbox click but image click gives detail...</label>
    <center>
    <div class="border border-outset" id="pic_1_border">
    <a href="../intermediates/AbbeyRoof (100).jpg" class="fancybox" rel="gallery" title="Sample title 1"><img src="../thumbnails/AbbeyRoof (100)_s.jpg" /></a>
<a target="_blank" href="../intermediates/AbbeyRoof (100).jpg">

</a>
    </div>
  </center>
  <script>
  const border_i = document.getElementById('pic_1_border').className;
  const cb_press = document.getElementById('cbox1');
  cb_press.addEventListener('click', function onClick(event) {

  if (document.getElementById('pic_1_border').className == "border border-inset") {
     document.getElementById('pic_1_border').className = "border border-outset";
  } else {
     document.getElementById('pic_1_border').className = "border border-inset";
  }
});
</script>
  </li>
  </td>

<td align="center">
  <li>
    <center>
    <input type="checkbox" name="bbb1[1]" value="../thumbnails/AbbeyRoof (101)_s.jpg" id="btn" checked="unchecked"/>Picture 2
  </center>
    <label for="original_1">what ever you want to say...</label>
    <center>
    <div class="box-1" id="box" style="font-size:16pt"> Selected<br />
    <a href="../intermediates/AbbeyRoof (101).jpg" class="fancybox" rel="gallery" title="Sample title 2"><img src="../thumbnails/AbbeyRoof (101)_s.jpg" /></a>
  </center>
  </div>
  <script>
  const box_i = document.getElementById('box');
  box_i.style.color = 'salmon';
  const btn = document.getElementById('btn');
  btn.addEventListener('click', function onClick(event) {
  const box = document.getElementById('box');

  if (box.style.color == 'skyblue') {
     box.style.color = 'salmon';
  } else {
     box.style.color = 'skyblue';
  }
});
</script>
  </li>
  </td>

<td align="center">
  <li>
    <div class="border border-groove">
    <center>
    <input type="checkbox" name="bbb1[2]" value="../thumbnails/AbbeyRoof (102)_s.jpg" checked="checked"/>Picture 3
    <a href="../intermediates/AbbeyRoof (102).jpg" class="fancybox" rel="gallery" title="Sample title 3"><img src="../thumbnails/AbbeyRoof (102)_s.jpg" /></a>
  </center>
  </div>
  </li>
</td>

<td align="center">
  <li>
    <center>
    <input type="checkbox" name="bbb1[3]" value="../thumbnails/AbbeyRoof (103)_s.jpg" checked="checked"/>Picture 4
  </center>
    <center>
    <div class="border border-ridge">picture shows good roof - grass in gutter
    <a href="../intermediates/AbbeyRoof (103).jpg" class="fancybox" rel="gallery" title="Sample title 4"><img src="../thumbnails/AbbeyRoof (103)_s.jpg" /></a>
<p style="border-color: red; border-style: solid; padding: 1rem">selected</p>
  </div>
  </center>
  </li>
</td>

<td align="center">
  <li>
    <center>
    <input type="checkbox" name="bbb1[4]" value="../thumbnails/AbbeyRoof (105)_s.jpg" checked="checked"/>Picture 5
  </center>
    <center>
    <div class="round round_border" id="round_border1">
    <a href="../intermediates/AbbeyRoof (105).jpg" class="fancybox" rel="gallery" title="Sample title 5"><img src="../thumbnails/AbbeyRoof (105)_s.jpg" /></a>
  </div>
  </center>
  </li>
  </td>
  </table>

  <center>
  <div class="border multi_color_box" id="">
  <p><input type="submit" value="submit"><br></br> Add Your Selection(s) to Database</p>
  </div>
  <div class="border multi_color_box" id="">picture is now submit button</div>
<table border="1" bgcolor="#CCDDFF"> <tr> <td bgcolor="red"> <p><input type="submit" name="action" value="AreaA" class="sub_img2"> </p> </td> <td> <p><input type="submit" name="action" value="AreaB" class="sub_img3"> </p>
</td> </tr>
<tr> <td> <p><input type="submit" name="action" value="AreaC" class="sub_img4"> </p>
 </td> <td bgcolor="#8888ff"> <p><input type="submit" name="action" value="AreaD" class="sub_img5"> </td> </tr> </table>
  <center>
</form>
</div>
</ul>

<script language="JavaScript" type="text/javascript">
<!--
function AllCheck2(){
  var check = document.form1.aa1.checked;
  document.form1.elements['bbb1[0]'].checked = check;
  document.form1.elements['bbb1[1]'].checked = check;
  document.form1.elements['bbb1[2]'].checked = check;
  document.form1.elements['bbb1[3]'].checked = check;
  document.form1.elements['bbb1[4]'].checked = check;
  document.form1.elements['bbb1[5]'].checked = check;
  const box_a = document.getElementById('box');
  /* if (box_a.style.color == 'skyblue') { */
  if (check == true) {
      box_a.style.color = 'salmon';
  } else {
     box_a.style.color = 'skyblue';
  }
  if (check == true) {
     document.getElementById('pic_1_border').className = "border border-outset";
  } else {
     document.getElementById('pic_1_border').className = "border border-inset";

  }
}
//-->
</script>
</body>
</html>
