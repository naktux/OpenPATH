import {cpSync,mkdirSync,writeFileSync,readFileSync,existsSync} from 'node:fs';import {join} from 'node:path';import {importJava,exportJava,newCall,uid} from '../app/model';
const root='2026-FRC-Rebuilt',target='output/compile-check';mkdirSync(target,{recursive:true});
for(const path of ['src','gradle','vendordeps','.wpilib','build.gradle','settings.gradle','gradlew.bat'])if(existsSync(join(root,path)))cpSync(join(root,path),join(target,path),{recursive:true});
const p=importJava(readFileSync('public/RobotContainer.java','utf8'));
if(process.argv.includes('--edited')){p.waypoints[0].x=4.42;p.waypoints[0].heading=37;p.waypoints.push({id:uid(),name:'OpenPathValidationPoint',x:2.1,y:1.1,heading:90});p.commands.push({id:uid(),name:'openPathValidationSequence',steps:[newCall('driveTo',['positions.OpenPathValidationPoint','0.1']),newCall('endIntake',[])]});p.routines.push({id:uid(),name:'OpenPATH Validation',isDefault:false,steps:[newCall('openPathValidationSequence',[])]});}
writeFileSync(join(target,'src/main/java/frc/robot/RobotContainer.java'),exportJava(p));console.log('Prepared isolated '+(process.argv.includes('--edited')?'edited':'unedited')+' export at '+target);
