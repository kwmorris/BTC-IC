/* �2001 coolmaps.com - http://club.coolmaps.com - clubmaster@coolmaps.com */var MB_brwsr;var MB_AN=navigator.appName;var MB_AV=navigator.appVersion;var MB_UA=navigator.userAgent;var MB_IE=(MB_AN=="Microsoft Internet Explorer")?1:0;var MB_NS=(MB_AN=="Netscape")?1:0;var MB_Mac=(MB_AV.indexOf("Mac")!=-1)?1:0;var MB_Ver=(MB_Mac && MB_IE)?parseInt(MB_AV.substring(22)):parseInt(MB_AV);var IE5m=(MB_IE && MB_Mac && MB_Ver>=5)?1:0;if((MB_IE && MB_Ver>=4 && !MB_Mac) || IE5m) MB_brwsr=0;else if(MB_NS && MB_Ver>=4 && MB_Ver<5) MB_brwsr=1;else if(MB_NS && MB_Ver>=5) MB_brwsr=2;var IE4s=(MB_brwsr==0 && !MB_Mac && parseFloat(MB_AV.substring(MB_AV.indexOf('MSIE')+5))<=5.01)?1:0;var MB_NS61=(MB_brwsr==2 && MB_UA.indexOf("6.1")!=-1)?true:false;function MB_getObj(r){switch(MB_brwsr){case 0: return document.all(r);case 1:if(r.indexOf("document")!=-1) return eval(r);else return document.layers[r];case 2:return document.getElementById(r);}}function MB_getWinWidth(){switch(MB_brwsr){case 0:return document.body.clientWidth+document.body.scrollLeft;case 1:case 2:return window.innerWidth+window.pageXOffset;}}function MB_getWinHeight(){switch(MB_brwsr){case 0:return document.body.clientHeight+document.body.scrollTop;case 1:case 2:return window.innerHeight+window.pageYOffset;}}function MB_getWidth(obj){switch(MB_brwsr){case 0:case 2:return obj.offsetWidth;case 1:return obj.clip.width;}}function MB_setWidth(obj,r){switch(MB_brwsr){case 0:case 2:obj.style.width=r;break;case 1:obj.clip.width=r;break;}}function MB_getHeight(obj){switch(MB_brwsr){case 0: case 2:return obj.offsetHeight;case 1:return obj.clip.height;}}function MB_setHeight(obj,r){switch(MB_brwsr){case 0:case 2: obj.style.height=r;break;case 1:obj.clip.height=r;break;}}function MB_getTop(obj){switch(MB_brwsr){case 0: case 2: var temp=obj.offsetParent;if(temp==null) return obj.offsetTop;else var x=obj.offsetTop+temp.offsetTop;while(temp!=null){temp=temp.offsetParent;if(temp!=null){var z=temp.offsetTop;x+=(Math.abs(z)==z)?z:0;}}if(!IE5m) return x;else{temp=obj.offsetParent;var x2=obj.style.posTop+temp.style.posTop;while(temp!=null){temp=temp.offsetParent;if(temp!=null){var z=temp.style.posTop;x2+=(Math.abs(z)==z)?z:0;}}return Math.max(x,x2);}case 1:return obj.pageY;}}function MB_setTop(obj,r){switch(MB_brwsr){case 0:case 2: obj.style.top=r;break;case 1:obj.top=r;break;}}function MB_getLeft(obj){switch(MB_brwsr){case 0: case 2:var temp=obj.offsetParent;if(temp==null) return obj.offsetLeft;else var x=obj.offsetLeft+temp.offsetLeft;while(temp!=null){temp=temp.offsetParent;if(temp!=null){var z=temp.offsetLeft;x+=(Math.abs(z)==z)?z:0;}}if(!IE5m)return x;else{temp=obj.offsetParent;var x2=obj.style.posLeft+temp.style.posLeft;while(temp!=null){temp=temp.offsetParent;if(temp!=null){var z=temp.style.posLeft;x2+=(Math.abs(z)==z)?z:0;}}return Math.max(x,x2);}case 1:return obj.pageX;}}function MB_setLeft(obj,r){switch(MB_brwsr){case 0:case 2: obj.style.left=r;break;case 1:obj.left=r;break;}}function MB_showItem(obj,a){if(a) obj=MB_getObj(obj);switch(MB_brwsr){case 0:case 2: obj.style.visibility="visible";break;case 1:obj.visibility="show";break;}}function MB_hideItem(obj){switch(MB_brwsr){case 0:case 2: obj.style.visibility="hidden";break;case 1:obj.visibility="hide";break;}}function MB_setZindex(obj,z){switch(MB_brwsr){case 0: case 2: obj.style.zIndex=z;break;case 1:obj.zIndex=z;break;}}function MB_sizeNav4(){var left=0;for(var i=0; i<this.menus.length; ++i){var width;var norm=this.getItem(0,i);var high=this.getItem(1,i);var dummy=this.getItem(2,i);var text=this.getItem(3,i);this.menus[i].base=this.reference+"_"+i+"_mi";var dd=dummy.document;dd.menu=eval(this.reference);dd.a=i;dd.b=-1;dd.onMouseUp=(this.openA)?MB_doActionNav6:MB_handleClickNav6;if(this.type==0 && i!=0)document.layers[this.reference].clip.width+=MB_getWidth(norm);else if(i!=0) document.layers[this.reference].clip.height+=MB_getHeight(norm);MB_setWidth(dummy,MB_getWidth(norm));MB_setHeight(dummy,MB_getHeight(norm));for(var r=0; r<this.menus[i].menuItems.length; ++r){var snorm=this.getItem(0,i,r);var shigh=this.getItem(1,i,r);var sdummy=this.getItem(2,i,r);var sd=sdummy.document;sd.menu=eval(this.reference);sd.a=i;sd.b=r;sd.onMouseUp=MB_doActionNav6;document.layers[this.reference+ "_"+i+"_mi"].clip.height+=MB_getHeight(snorm);MB_setWidth(sdummy,MB_getWidth(snorm));MB_setHeight(sdummy,MB_getHeight(snorm)); }this.stackItems(i);}if(this.type==0)this.tileItems();else this.stackItems();}function MB_sizeIE4s(){var left=0;for(var i=0; i<this.menus.length; ++i){var divTag=this.getItem(0,i);var text=MB_getObj(this.reference+"_"+i+"_text");MB_setWidth(divTag,MB_getWidth(text)+6+(this.bwidth*2));MB_setHeight(divTag,MB_getHeight(text)+6+(this.bwidth*2));MB_setLeft(divTag,left);left+=MB_getWidth(text)+6+this.bwidth;}}function MB_stackItems(b){var top=0;var obj;if(b==null) obj=this.menus;else obj=this.menus[b].menuItems;for(var i=0; i<obj.length; ++i){var norm=(b==null)?this.getItem(0,i):this.getItem(0,b,i);var high=(b==null)?this.getItem(1,i):this.getItem(1,b,i);var dummy=(b==null)?this.getItem(2,i):this.getItem(2,b,i);MB_setTop(norm,top);MB_setTop(high,top);MB_setTop(dummy,top);top+=MB_getHeight(norm)-this.bwidth;}}function MB_tileItems(){var left=0;for(var i=0; i<this.menus.length; ++i){var norm=this.getItem(0,i);var high=this.getItem(1,i);var dummy=this.getItem(2,i);var text=this.getItem(3,i);MB_setLeft(norm,left);MB_setLeft(high,left);MB_setLeft(dummy,left);left+=MB_getWidth(norm)-this.bwidth;}}function MB_getItem(c,a,b){var rv;if(MB_brwsr!=1){if(b==null)rv=MB_getObj(this.reference+"_"+a);else rv=MB_getObj(this.reference+"_"+a+"_mi"+"_"+b);}else{if(b==null)rv=this.menus[a].objRef[c];else rv=this.menus[a].menuItems[b].objRef[c];}return rv;}function MB_writeStyles(){this.fsize+=1;var bw='border-width: '+this.bwidth+';';var bs='border-style: '+this.bstyle+';';var pd='padding: 3;';var ff='font-family: '+this.fface+';';var fs='font-style: '+this.fstyle+';';var fw='font-weight: '+this.fweight+';';var fz='font-size: '+this.fsize+'px;';var styleDef='<STYLE>';styleDef+='.norm'+this.reference+' {';styleDef+=bw+'border-color: '+this.bncolor+';'+bs+'}';styleDef+='.high'+this.reference+' {';styleDef+=bw+'border-color: '+this.bncolor+';'+bs+'}';styleDef+='.normText'+this.reference+' {'+'color: '+this.fncolor+';'+ff+fz+fs+fw;if(this.bwidth==0) styleDef+=pd;styleDef+='}';styleDef+='.highText'+this.reference+' {'+'color: '+this.fhcolor+';'+ff+fz+fs+fw;if(this.bwidth==0) styleDef+=pd;styleDef+='}';styleDef+='</STYLE>';document.write(styleDef);}function MenuBar(reference,type,opens,mwidth,offset,mialign,openA,bwidth,bstyle,bcolor,bgncolor,bghcolor,fface,fsize,fstyle,fweight,fncolor,fhcolor,dom){this.reference=reference;this.type=type;this.opens=opens;this.mwidth=mwidth;this.offset=offset;this.mialign=mialign;this.openA=openA;this.stop=-1;this.bwidth=bwidth;this.bstyle=bstyle;this.bncolor=bcolor;this.bgncolor=bgncolor;this.bghcolor=bghcolor;this.fface=fface;this.fsize=(MB_NS && MB_Mac)?fsize-1:fsize;this.fstyle=fstyle;this.fweight=fweight;this.fncolor=fncolor;this.fhcolor=fhcolor;this.dom=dom;this.menus=null;this.menuClicked=-1;this.x=0;this.y=0;this.addMenu=MenuBar_addMenu;this.start=MenuBar_display;this.displayIENav6=MenuBar_displayIENav6;this.displayNav4=MenuBar_displayNav4;this.showMenu=MenuBar_showMenu;this.hideMenu=MenuBar_hideMenu;this.MBmouseOver=MB_mouseOver;this.MBmouseOut=MB_mouseOut;this.highlight=MB_highlight;this.normal=MB_normal;this.doAction=MB_doAction;this.handleClick=MenuBar_handleClick;if(MB_brwsr==1){this.sizeNav4=MB_sizeNav4;this.writeStyles=MB_writeStyles;this.stackItems=MB_stackItems;this.tileItems=MB_tileItems;this.writeStyles();}else if(MB_brwsr==0) this.sizeIE4s=MB_sizeIE4s;this.getItem=MB_getItem;this.eventMove=MB_eventMove;if(IE4s) this.moveIE4=MB_moveIE4;this.addonResize=MenuBar_addonResize;this.menuHeight=MB_menuHeight;}function MenuBar_addMenu(menu){if(this.menus==null){this.menus=new Array(1);this.menus[0]=menu;}else{this.menus[this.menus.length]=menu;}}function MenuBar_onLoad(){eval(window.coolOLTemp);}function MenuBar_addonResize(){if(window.coolORTemp==null){window.coolORTemp=""+window.onresize;window.coolORTemp=window.coolORTemp.substring(window.coolORTemp.indexOf("{")+1,window.coolORTemp.lastIndexOf("}")-1);}if(MB_brwsr!=1) window.coolORTemp+=";"+this.reference+".eventMove();";else window.coolORTemp="window.location.reload();";window.onresize=MenuBar_onResize;}function MenuBar_onResize(){eval(window.coolORTemp);}function MB_eventMove(){var IE4=(parseInt(MB_AV.substring(MB_AV.indexOf('MSIE')+5))==4)?1:0;var obj1=MB_getObj(this.reference);var obj2;if(MB_brwsr==1) obj2=MB_getObj("document."+this.dom+"MBpos"+this.reference+"L");else obj2=MB_getObj("MBpos"+this.reference);var x;var y;if(obj2==null){x=this.x;}else{switch(MB_brwsr){case 0: case 2:if(IE4) setTimeout(this.reference+".moveIE4()",25);else{x=MB_getLeft(obj2);y=MB_getTop(obj2);}break;case 1:x=obj2.pageX;y=obj2.pageY;break;}}this.x=x;this.y=y;if(!IE4){if(MB_brwsr!=1 && this.mwidth && !IE4s) y+=3+this.bwidth;MB_setLeft(obj1,x);MB_setTop(obj1,y);MB_showItem(obj1);}}function MB_moveIE4(){var x=MB_getLeft(MB_getObj("MBpos"+this.reference));var y=MB_getTop(MB_getObj("MBpos"+this.reference));var obj=MB_getObj(this.reference);this.x=x;this.y=y;MB_setLeft(obj,x);MB_setTop(obj,y);MB_showItem(obj);}function MenuBar_display(){if(MB_brwsr!=null){var tr=this.reference;if(window.coolOLTemp==null){window.coolOLTemp=""+window.onload;window.coolOLTemp=window.coolOLTemp.substring(window.coolOLTemp.indexOf("{")+1,window.coolOLTemp.lastIndexOf("}")-1);}if(MB_brwsr==1) window.coolOLTemp+=";"+tr+".menuHeight();"+tr+".eventMove();setTimeout(\""+tr+".addonResize();\",1000);";else window.coolOLTemp+=";"+tr+".menuHeight();"+tr+".eventMove();"+tr+".addonResize();";window.onload=MenuBar_onLoad;switch(MB_brwsr){case 0: case 2:this.displayIENav6();if(IE4s && !this.type && this.mwidth) this.sizeIE4s();if(IE4s) MB_getObj(tr).style.width=0;break;case 1: this.displayNav4();this.sizeNav4();break;}}}function MenuBar_displayIENav6(){var styleString='position:absolute;left:0;top:0;z-index:497;visibility:hidden;';document.write('<DIV ID="'+this.reference+'" STYLE="'+styleString+'">');var r=0;for(var i=0; i<this.menus.length; ++i) r=this.menus[i].displayIENav6(this.reference,i,r);document.write('</DIV>');for(var i=0; i<this.menus.length; ++i) this.menus[i].displayMenuItemsIENav6(this.reference,i);}function MenuBar_displayNav4(){document.writeln('<LAYER ID="'+this.reference+'" PAGEX="0" PAGEY="0" Z-INDEX=497 VISIBILITY="hide">');for(var i=0; i<this.menus.length; ++i) this.menus[i].displayNav4(this.reference,i);document.writeln('</LAYER>');for(var i=0; i<this.menus.length; ++i) this.menus[i].displayMenuItemsNav4(this.reference,i);}function MB_highlight(a,b){if(MB_brwsr==1){if(b==-1) MB_setZindex(this.getItem(1,a),501);else MB_setZindex(this.getItem(1,a,b),501);}else{var bc=this.bghcolor;var it;if(b==-1) it=this.getItem(1,a);else it=this.getItem(1,a,b);with(it.style){if(bc!="") backgroundColor=bc;color=this.fhcolor;}}}function MB_normal(a,b){if(MB_brwsr==1){if(b==-1) MB_setZindex(this.getItem(1,a),499);else MB_setZindex(this.getItem(1,a,b),499);}else{var bc=this.bgncolor;var it;if(b==-1) it=this.getItem(1,a);else it=this.getItem(1,a,b);with(it.style){if(bc!="") backgroundColor=bc;color=this.fncolor;}}}function MB_mouseOver(a,b){this.stop=a;if(b==-1){if(this.openA){if(this.menuClicked!=a && this.menuClicked!=-1){this.normal(this.menuClicked,-1);MB_hideItem(MB_getObj(this.menus[this.menuClicked].base));}this.showMenu(a);this.menuClicked=a;}}this.highlight(a,b);}function MB_mouseOut(a,b){if(this.openA){this.stop=-1;setTimeout(this.reference+".hideMenu("+a+")",50);}if(b!=-1) this.normal(a,b);else if(b==-1 && !this.openA && this.menuClicked!=a) this.normal(a,b);}function MB_doActionNav6(){this.menu.doAction(this.a,this.b);}function MB_doAction(a,b){if(b==-1) obj=this.menus[a];else obj=this.menus[a].menuItems[b];if(obj.action.length>0){if(obj.action.indexOf("javascript:")==0) eval(obj.action);else if(obj.target!="") window.open(obj.action,obj.target);else parent.location.href=obj.action;MB_setZindex(this.getItem(1,a),499);MB_hideItem(MB_getObj(this.menus[a].base));this.menuClicked=-1;}}function MB_handleClickNav6(){this.menu.handleClick(this.a,this.b);}function MenuBar_handleClick(a,b){if(this.menuClicked>=0){if(a==this.menuClicked){this.normal(this.menuClicked,-1);MB_hideItem(MB_getObj(this.menus[this.menuClicked].base));this.menuClicked=-1;}else{MB_hideItem(MB_getObj(this.menus[this.menuClicked].base));this.normal(this.menuClicked,-1);this.showMenu(a);this.menuClicked=a;} }else{this.showMenu(a);this.highlight(a,-1);this.menuClicked=a;}}function MenuBar_hideMenu(n){if(this.stop!=n){this.normal(n,-1);MB_hideItem(MB_getObj(this.menus[n].base));}}function MB_menuHeight(){for(var i=0; i<this.menus.length; i++){if(MB_brwsr==1) for(var e=0; e<this.menus[i].menuItems.length; e++) this.menus[i].menuHeight+=MB_getHeight(this.getItem(0,i,e));else this.menus[i].menuHeight=MB_getHeight(MB_getObj(this.menus[i].base));}}function MenuBar_showMenu(n){var menuBarDiv=MB_getObj(this.reference);var menuHeadDiv=this.getItem(2,n);var menuItemDiv=MB_getObj(this.menus[n].base);if(IE4s) MB_setWidth(menuItemDiv,this.menus[n].menuwidth);var itemWidth=0;var menuBarX;var menuBarY;if(this.menus[n].menuItems.length>0) itemWidth=MB_getWidth(this.getItem(0,n,0));if(MB_NS61 && n!=0) itemWidth-=this.bwidth;if(this.opens==0 || this.opens==2){var headWidth=MB_getWidth(this.getItem(2,n));menuBarX=MB_getLeft(this.getItem(2,n));if((MB_brwsr==2 && !MB_NS61) || (MB_NS61 && n!=0 && this.mwidth==1)) menuBarX-=this.bwidth;if(MB_getWinWidth()<menuBarX+itemWidth+20) menuBarX+=(headWidth-itemWidth);else if(MB_brwsr==0 && n!=0 && this.mwidth && !IE4s) menuBarX-=this.bwidth;if(this.opens==0){menuBarY=0;if(this.menus[n].menuItems.length>0){var tempDiv=this.getItem(2,n,this.menus[n].menuItems.length-1);menuBarY=this.y-this.menus[n].menuHeight+(((MB_brwsr==1 || !this.mwidth)?0:this.bwidth+3)+this.bwidth*this.menus[n].menuItems.length);}}else{menuBarY=this.y+MB_getHeight(menuHeadDiv)-this.bwidth+((MB_brwsr==1 || !this.mwidth)?0:this.bwidth+3);}if(MB_brwsr!=1 && this.mwidth && !IE4s && !IE5m) menuBarY-=3+this.bwidth;}else if(this.opens==1 || this.opens==3){menuBarY=MB_getTop(menuHeadDiv)+this.offset;if(MB_NS61) menuBarY+=this.bwidth;if(MB_brwsr==2) menuBarY-=this.bwidth;var itemHeight=this.menus[n].menuHeight;if(MB_getWinHeight()<menuBarY+itemHeight) menuBarY-=(menuBarY+itemHeight)-MB_getWinHeight();if(this.opens==1) menuBarX=this.x+MB_getWidth(menuHeadDiv)-this.bwidth-this.offset;else menuBarX=this.x-itemWidth+this.bwidth+this.offset;}if(IE5m){if(this.type==1){menuBarY-=3+this.bwidth;}if(this.type==0){var ma=3;if(n==0 || this.mwidth==0)ma+=this.bwidth;if(this.mwidth==1) menuBarY-=3+this.bwidth;menuBarX-=ma;}}if(IE4s && this.mwidth==1) menuBarY-=3+this.bwidth;MB_setLeft(menuItemDiv,menuBarX);MB_setTop(menuItemDiv,menuBarY);MB_showItem(menuItemDiv);}function Menu(name, headwidth, menuwidth, action, target){this.name=name;this.headwidth=headwidth;this.menuwidth=menuwidth;this.action=(action!=null)?action:"";this.target=(target!=null)?target:"";this.menuItems=0;this.menuHeight=0;this.addMenuItem=Menu_addMenuItem;this.makeItem=MB_makeItem;this.displayIENav6=Menu_displayIENav6;this.displayNav4=Menu_displayNav4;this.displayMenuItemsIENav6=Menu_displayMenuItemsIENav6;this.displayMenuItemsNav4=Menu_displayMenuItemsNav4;}function Menu_addMenuItem(name,action,target){var menuItem=new MenuItem(name,action,target);if(this.menuItems==0){this.menuItems=new Array(1);this.menuItems[0]=menuItem;}else{this.menuItems[this.menuItems.length]=menuItem;}}function MB_makeItem(reference,menuNumber,r,b,c){var menu=eval(reference);var id;var name;var width;switch(b){case 1:id=reference+"_"+menuNumber+"_mi_"+c;name=this.menuItems[c].name;width=this.menuwidth;break;case 0:id=reference+"_"+menuNumber;name=this.name;width=this.headwidth;break;}var align='center';if(b){switch(menu.mialign){case 0: align='left';break;case 2: align='right';break;}}switch(MB_brwsr){case 0: case 2:var openA=reference+'.doAction('+menuNumber+','+c+')';if(!menu.openA && !b) openA=reference+'.handleClick('+menuNumber+',-1)';var html="";if((menu.mwidth && IE4s) || !menu.mwidth || b){html+='<DIV ID="'+id+'" STYLE="';if(menu.type || b){html+='position:relative;';html+='top:'+r+'px; ';r-=(b && menu.mwidth)?menu.bwidth+1:menu.bwidth;}else if(!menu.type){html+='position:absolute;';html+='left:'+r+'px; ';r+=this.headwidth+menu.bwidth;}if(MB_brwsr==2) html+='width:'+(width-6)+'px;';else if(b || !IE4s || !menu.mwidth) html+='width:'+(width+menu.bwidth*2)+'px;';html+='text-align:'+align+';';html+='border-width:'+menu.bwidth+'px;';}else if(menu.mwidth && !b){html+='<SPAN ID="'+id+'" STYLE="position: relative;';html+='border-top-width:'+menu.bwidth+'px;';html+='border-right-width:'+menu.bwidth+'px;';html+='border-bottom-width:'+menu.bwidth+'px;';html+=(r==0 || IE4s)?'border-left-width:'+menu.bwidth+'px;':'border-left-width: 0px;';}if(menu.bgncolor!="") html+='background-color:'+menu.bgncolor+';';html+='border-style:solid;';html+='border-color:'+menu.bncolor+';';html+='padding:3px;';html+='font-family:'+menu.fface+';';html+='font-size:'+menu.fsize+'px;';html+='font-weight:'+menu.fweight+';';html+='font-style:'+menu.fstyle+';';html+='color:'+menu.fncolor+';';html+='cursor:default;" ';html+='onMouseOver="'+reference+'.MBmouseOver('+menuNumber+','+c+')" ';html+='onMouseOut="'+reference+'.MBmouseOut('+menuNumber+','+c+')" ';html+='onClick="'+openA+'">';if(IE4s && menu.mwidth && !b) html+='<SPAN ID="'+id+'_text">';html+=name;if(menu.mwidth && !b) html+='</SPAN>';if((menu.mwidth && IE4s) || !menu.mwidth || b) html+='</DIV>';if(menu.mwidth) r+=1;break;case 1:var widths;if(!menu.mwidth || b) widths=(menu.bwidth==0)?width:width-6;else width=0;var a1='<LAYER ID="'+id;var a2=reference+'" BGCOLOR="';var a3='" Z-INDEX=';var a4='<TABLE BORDER=0 CELLPADDING=0 CELLSPACING=0 BGCOLOR="';var a5='" WIDTH="'+widths+'">';var a6='<TR><TD><DIV class="';var a7='Text'+reference+'" ALIGN="'+align+'">'+name+'</DIV></TD></TR></TABLE></LAYER>'; var html=a1+'_norm" class="norm'+a2+menu.bgncolor+a3+'500>'+a4+menu.bgncolor+a5+a6+'norm'+a7; html+=a1+'_high" class="high'+a2+menu.bghcolor+a3+'499>'+a4+menu.bghcolor+a5+a6+'high'+a7;html+=a1+'_dummy" class="dummy'+reference+'" Z-INDEX=502 onMouseOver="window.'+reference+'.MBmouseOver('+menuNumber+','+c+')" onMouseOut="window.'+reference+'.MBmouseOut('+menuNumber+','+c+')">';html+='</LAYER>';break;}document.write(html);return r;}function Menu_displayIENav6(reference,menuNumber,r){return this.makeItem(reference,menuNumber,r,0,-1);}function Menu_displayMenuItemsIENav6(reference,menuNumber){var id=reference+"_"+menuNumber+"_mi";var style="position:absolute;top:0;left:0;visibility:hidden;z-index:498";var menu=eval(reference);this.base=id;document.write('<DIV ID="'+id+'" STYLE="'+style+'">');var r=0;for(var i=0; i<this.menuItems.length; ++i){r=this.makeItem(reference,menuNumber,r,1,i); }document.write('</DIV>');}function Menu_displayNav4(reference,menuNumber){var id=reference+"_"+menuNumber;var r=this.makeItem(reference,menuNumber,0,0,-1);this.objRef=new Array(eval("document."+reference+".document."+id+"_norm"),eval("document."+reference+".document."+id+"_high"),eval("document."+reference+".document."+id+"_dummy"),"");}function Menu_displayMenuItemsNav4(reference,menuNumber){var id=reference+"_"+menuNumber+"_mi";var base="document."+id+".document.";document.write('<LAYER ID="'+id+'" PAGEX="10 PAGEY="10" VISIBILITY="hidden Z-INDEX=498">');for(var i=0; i<this.menuItems.length; ++i){var r=this.makeItem(reference,menuNumber,0,1,i);this.menuItems[i].objRef=new Array(eval(base+id+"_"+i+"_norm"),eval(base+id+"_"+i+"_high"),eval(base+id+"_"+i+"_dummy"));}document.writeln('</LAYER>');}function MenuItem(name,action,target){this.name=name;this.action=action;this.target=(target!=null)?target:"";}