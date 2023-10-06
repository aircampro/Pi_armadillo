        var RefMenuMode = "";
        function GetParam() {
          RefMenuMode = "";
          /* 繧｢繝峨Ξ繧ｹ縺ｮ縲�?縲堺ｻ･髯阪�蠑墓焚(繝代Λ繝｡繝ｼ繧ｿ)繧貞叙蠕� */
          var pram = top.location.href;
          /* 蜈磯�ｭ縺九ｉ?縺ｾ縺ｧ繧偵き繝�ヨ */
          pram = pram.substring(pram.indexOf('?') + 1);
          if (pram.indexOf('#') != -1) {
             pram = pram.substring(0, pram.indexOf('#'));
          }
          //window.alert("pram = "+pram);
          /* 蠑墓焚縺後↑縺�凾縺ｯ蜃ｦ逅�＠縺ｪ縺� */
          if (!pram) {
              RefMenuMode = "dtald";
             return false;
          } else {
             /* 縲�&縲阪〒蠑墓焚繧貞�蜑ｲ縺励※驟榊�縺ｫ */
             var pair = pram.split("&");
             //window.alert("pair = " + pair);
             var i = temp = "";
             var key = new Array();
             var keyName = keyValue = "";
             for (i = 0; i < pair.length; i++) {
               /* 驟榊�縺ｮ蛟､繧偵�=縲阪〒蛻�牡 */
               temp = pair[i].split("=");
               keyName = temp[0];
               keyValue = temp[1];
               /* 繧ｭ繝ｼ縺ｨ蛟､縺ｮ騾｣諠ｳ驟榊�繧堤函謌� */
               key[keyName] = keyValue;
             }
             /* 繝｡繝九Η繝ｼ繝｢繝ｼ繝� */
             if (key["menutype"] != "") {
               RefMenuMode = unescape(key["menutype"]);
             } else {
                 RefMenuMode = "dtald";
             }
          }
          //window.alert("RefMenuMode = " + RefMenuMode);
        }
        function SetFrameSrc() {
           //window.alert("top.location.pathname = " + top.location.pathname);
           var UrlStr1 = top.location.pathname.toLowerCase() ;
            //window.alert("UrlStr1 = " + UrlStr1);
           var FileNam1 = UrlStr1.slice(UrlStr1.lastIndexOf('/')+1) ;
            //window.alert("FileNam1 = " + FileNam1);
           var iDir = "";
           if (FileNam1.indexOf('_css.php') != -1 || FileNam1 == 'css3.html'
            || FileNam1.indexOf('_how2.html') != -1) {
                 iDir = "../" ;
           } else {
                 iDir = "" ;
           }
            //window.alert("iDir = " + iDir);
           GetParam();
           var obj = document.getElementById("ReferenceMenu");
            //window.alert("obj = " + obj);
           if (RefMenuMode.indexOf('simpl') != -1) {
               obj.src = iDir + "html5_css3_menu_s.html";
           } else if (RefMenuMode.indexOf('rvers') != -1) {
               obj.src = iDir + "html5_css3_menu_r.html";
           } else {
               obj.src = iDir + "html5_css3_menu.html";
           }
           //window.alert("obj.src = " + obj.src);
        }

        function go2top(strID) {                                      /* 繝代Λ繝｡繝ｼ繧ｿ繧堤ｶｭ謖√＠縺溘∪縺ｾ 繝壹�繧ｸ繝医ャ繝励↓遘ｻ蜍� */
           var UrlStr = top.location.href;
           if (strID === undefined) {
               strID = "";
           }
           if (UrlStr.indexOf('#') != -1) {
              UrlStr = UrlStr.substring(0, UrlStr.indexOf('#'));
           }
           if (strID !== "") {
              UrlStr = UrlStr+"#"+strID;
           }
           /* 迴ｾ蝨ｨ縺ｮ繝｡繝九Η繝ｼ驕ｸ謚樒憾諷九ｒ蜿門ｾ� */
           var refM = top.document.getElementById('ReferenceMenu');
           refM = refM.getAttribute('name');
           if (refM.indexOf('&') != -1) {
              var MenuPos = refM.slice(refM.indexOf('&') + 1);
           }
           var scrollT1 = document.documentElement.scrollTop || document.body.scrollTop;
           top.location.href = UrlStr;
           /* 繝｡繝九Η繝ｼ菴咲ｽｮ縺後悟崋螳壹阪〒縲�#PageBody縲堺ｻ･螟悶�繝上ャ繧ｷ繝･縲�#縲阪ｒ繧ゅ▽縺ｨ縺� */
           if (MenuPos === '0' && strID.length>=1 && strID !== "PageBody") {
              var scrollT2 = document.documentElement.scrollTop || document.body.scrollTop;
              /* 繧ｯ繝ｪ繝�け縺励◆菴咲ｽｮ繧医ｊ荳翫�菴咲ｽｮ縺ｸ縺ｮ遘ｻ蜍墓凾縺ｮ縺ｿ陦ｨ遉ｺ菴咲ｽｮ繧定｣懈ｭ｣縺吶ｋ */
              if ((scrollT1 - scrollT2) > 0 ) {     
                 window.scrollBy(0,-260);
              }
           }
           return false;
        }


        function localLink(linkTarget) {                             /* 繝代Λ繝｡繝ｼ繧ｿ繧堤ｶｭ謖√＠縺溘∪縺ｾ 謖�ｮ壹�繝ｼ繧ｸ縺ｫ遘ｻ蜍� */
          /* 迴ｾ蝨ｨ縺ｮ繝｡繝九Η繝ｼ驕ｸ謚樒憾諷九ｒ蜿門ｾ� */
          var refM = top.document.getElementById('ReferenceMenu');
          refM = refM.getAttribute('name');
          //window.alert("refM = " + refM);
          if (refM.indexOf('&') != -1) {
              var MenuTypeI = refM.slice(0,1);
              if (MenuTypeI==='0' || MenuTypeI==='1' || MenuTypeI==='2') { /* 繝舌げ菫ｮ豁｣縺ｾ縺ｧ縺ｮ閾ｨ譎ょ�鄂ｮ */
                 var MenuType = refM.slice(1, refM.indexOf('&'));
              } else {
                 var MenuType = refM.slice(0, refM.indexOf('&'));
              }
              //window.alert("MenuType = " + MenuType);
              if (MenuType.length < 11) {
                 MenuType += 'l01l02l03A0';
              }
              var MenuPos = refM.slice(refM.indexOf('&') + 1);
              //window.alert("MenuPos = " + MenuPos);
          } else {
              var MenuType = 'dtaldl01l02l03A0';
              var MenuPos = '2';
          }
          /* 繧｢繝峨Ξ繧ｹ縺ｮ縲�?縲堺ｻ･髯阪�蠑墓焚(繝代Λ繝｡繝ｼ繧ｿ)繧貞叙蠕� */
          var pramMenu = top.location.href;
          if (pramMenu === undefined) {
               return false;
          }
          /* 蠑墓焚繧呈欠螳壹＠縺ｪ縺�他縺ｳ蜃ｺ縺励�縺ｨ縺� */
          if (linkTarget === undefined) {
             linkTarget = top.location.href;
          /* 蠑墓焚縺梧欠螳壹＆繧後◆蜻ｼ縺ｳ蜃ｺ縺励→縺� */
          } else {
             /* 蠑墓焚縺ｫ繝壹�繧ｸ蜀��繝ｪ繝ｳ繧ｯ縺梧欠螳壹＆繧後※縺�ｋ縺ｨ縺� */
             if (linkTarget.indexOf('#') != -1) {
                linkID = linkTarget.substring(linkTarget.indexOf('#'));
                //window.alert("linkID = " + linkID);
                linkTarget = linkTarget.substring(0, linkTarget.indexOf('#'));
                //window.alert("linkTarget = " + linkTarget);
             /* 蠑墓焚縺ｫ繝壹�繧ｸ蜀��繝ｪ繝ｳ繧ｯ縺梧欠螳壹′縺ｪ縺上√縲縲 */
             /* 迴ｾ蝨ｨ縺ｮ繝｡繝九Η繝ｼ險ｭ螳壹′縲後せ繧ｭ繝��縲阪〒縺ｪ縺�→縺� */
             } else if (MenuPos === '1') {
                 linkID = "#PageBody";
             } else {
                 linkID = "";
             }
             linkTarget = linkTarget + "?menutype=" + MenuPos + MenuType + linkID;
             // linkTarget = linkTarget + pramMenu +linkID;
          }
          top.location.href = linkTarget;
          /* linkTarget縺ｮ繝峨く繝･繝｡繝ｳ繝医′繝ｭ繝ｼ繝峨＆繧後↑縺�→縺阪�縺ｿ莉･荳九′螳溯｡後＆繧後ｋ */
          if (MenuPos === '0' && linkID.length>1 && linkID !== "#PageBody") {
             window.scrollBy(0,-260);
          }
          return false;
        }

