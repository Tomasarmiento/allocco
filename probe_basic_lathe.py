#!/usr/bin/env python

import os
import sys
import importlib.util
import time, datetime

import linuxcnc
import hal, hal_glib
import threading

from qtpy.QtCore import Slot, QRegExp
from PyQt5.QtCore import QThread, pyqtSignal
from qtpy.QtGui import QFontDatabase, QRegExpValidator
from qtpyvcp.actions.machine_actions import issue_mdi
from qtpy.QtWidgets import QAbstractButton

from qtpyvcp import actions
from qtpyvcp.utilities import logger
from qtpyvcp.widgets.form_widgets.main_window import VCPMainWindow

from . import probe_basic_lathe_rc

LOG = logger.getLogger('QtPyVCP.' + __name__)
VCP_DIR = os.path.abspath(os.path.dirname(__file__))
INIFILE = linuxcnc.ini(os.getenv("INI_FILE_NAME"))

# Add custom fonts
QFontDatabase.addApplicationFont(os.path.join(VCP_DIR, 'fonts/BebasKai.ttf'))

class ProbeBasicLathe(VCPMainWindow):
    """Main window class for the ProbeBasic VCP."""
    def __init__(self, *args, **kwargs):
        super(ProbeBasicLathe, self).__init__(*args, **kwargs)
        self.run_from_line_Num.setValidator(QRegExpValidator(QRegExp("[0-9]*")))
        self.feed_unit_per_minute = 0.0
        self.feed_per_rev = 0.0
        self.h = hal.component("probe_basic_lathe")
        self.css_sword = 0.0
        self.rpm_mode = 0.0
        self.TIMEOUT_PNEUMATIC = 8
        self.wait_time = 0.2
        self.err_routine = True
        self.hglib_pin = hal_glib.GPin
        self.btnMdiBksp.clicked.connect(self.mdiBackSpace_clicked)
        self.btnMdiSpace.clicked.connect(self.mdiSpace_clicked)
        self.load_user_tabs()
        self.chk_init_conditions = False
        self.init_conditions_error_messages = {
            'emergency': [],
            'motorized_tool': [],
            'gama_changer_1': [],
            'torno_entrar': [],
            'torno_salir': [],
            'pateador': []
        }
        self.routine_error_messages = {
            'emergency': [],
            'motorized_tool': [],
            'gama_changer_1': [],
            'torno_entrar': [],
            'torno_salir': [],
            'pateador': []
        }
        self.active_threads = {
            'gama_changer_1': [],
            'init_cycle': [],
        }

        self.actual_gear = ""

        self.py_out_pins = {
                            'Cmd_CNC_OK': 0,
                            'Cmd_Pres1_Mord': 0,
                            'Cmd_CNC_OK_Man': 0,
                            'Cmd_Pres2_Mord': 0,
                            'Cmd_Consigna_TorreMot': 0,
                            'Cmd_Cont_Arm_EjeC': 0,
                            'Cmd_Giro_Hus_M3M4': 0,
                            'Cmd_Hab_Reg_EjeC': 0,
                            'Cmd_Vel_CambioGamas': 0,
                            'Cmd_Lim_Corr_EjeC': 0,
                            'Cmd_Jog_Cab': 0,
                            'Cmd_Hab_Reg_XZ': 0,
                            'Cmd_EV_Gama_M43': 0,
                            'Cmd_Lib_Freno_X': 0,
                            'Cmd_EV_Gamas_M41M43': 0,
                            'Cmd_Arm_Mot_Torre': 0,
                            'Cmd_Cont_Pot_XZ': 0,
                            'Cmd_EV_Desbloq_Torre': 0,
                            'Cmd_Lib_Freno_Cab_EjeC': 0,
                            'Cmd_EV_Refrig1_M8': 0,
                            'Cmd_EV_Gama_M41': 0,
                            'Cmd_EV_Refrig2_M12': 0,
                            'Cmd_EV_Gamas_M42M43': 0,
                            'Cmd_Bomba_Refrig': 0,
                            'Cmd_EV_Cerrar_Mord': 0,
                            'Cmd_EV_Acop_GiroTorre': 0,
                            'Cmd_Hab_Reg_Torre': 0,
                            'Cmd_EV_Acop_Contra': 0,
                            'Cmd_Gama_I_HerrMot': 0,
                            'Cmd_EV_Desbloq_Contra': 0,
                            'Cmd_Gama_II_HerrMot': 0,
                            'Cmd_EV_CanaContra_Adel': 0,
                            'Cmd_ValorTeo_Girar': 0,
                            'Cmd_EV_CanaContra_Atras': 0,
                            'Cmd_EV_Desac_EjeC': 0,
                            'Cmd_Marcha_Extr_Virutas': 0,
                            'Cmd_EV_Acop_EjeC': 0,
                            'Cmd_ContraMarch_ExtrViru': 0,
                            'Cmd_EV_PresReduc_EjeC': 0,
                            'Cmd_EV_Lub_Guias': 0,
                            'Cmd_Lib_Avan_Ref': 0,
                            'Cmd_EV_Cerrar_Luneta1': 0,
                            'Cmd_EV_Cerrar_Luneta2': 0,
                            'Cmd_EV_PresAlta_EjeC': 0,
                            'Cmd_EV_Abrir_Luneta1': 0,
                            'Cmd_EV_Abrir_Mord': 0,
                            'Cmd_EV_Desbloq_Luneta': 0,
                            'Cmd_Mon_Vel': 0,
                            'Cmd_EV_{Cerrar_Luneta2': 0,
                            'Cmd_EV_Abrir_Luneta2': 0,
                            'Cmd_Deshab_Campo': 0,
							}
        
        self.py_mcodes_pins = {
                            'PYM0' : 'motion.digital-out-00',
                            'PYM1' : 'motion.digital-out-01',
                            'PYM2' : 'motion.digital-out-02',
                            'PYM3' : 'motion.digital-out-03',
                            'PYM4' : 'motion.digital-out-04',
                            'PYM5' : 'motion.digital-out-05',
                            'PYM6' : 'motion.digital-out-06',
                            'PYM7' : 'motion.digital-out-07',
                            'PYM8' : 'motion.digital-out-08',
                            'PYM9' : 'motion.digital-out-09',
                            'PYM10' : 'motion.digital-out-10',
                            'PYM11' : 'motion.digital-out-11',
                            'PYM12' : 'motion.digital-out-12',
                            'PYM13' : 'motion.digital-out-13',
                            'PYM14' : 'motion.digital-out-14',
                            'PYM15' : 'motion.digital-out-15',
                            'PYM16' : 'motion.digital-out-16',
                            'PYM17' : 'motion.digital-out-17',
                            'PYM18' : 'motion.digital-out-18',
                            'PYM19' : 'motion.digital-out-19',
                            'PYM20' : 'motion.digital-out-20',
                            'PYM21' : 'motion.digital-out-21',
                            'PYM22' : 'motion.digital-out-22',
                            'PYM23' : 'motion.digital-out-23',
                            'PYM24' : 'motion.digital-out-24',
                            'PYM25' : 'motion.digital-out-25',
                            'PYM26' : 'motion.digital-out-26',
                            'PYM27' : 'motion.digital-out-27',
                            'PYM28' : 'motion.digital-out-28',
                            'PYM29' : 'motion.digital-out-29',
                            'PYM30' : 'motion.digital-out-30',
                            'PYM31' : 'motion.digital-out-31',
                            'PYM32' : 'motion.digital-out-32',
                            'PYM33' : 'motion.digital-out-33',
                            'PYM34' : 'motion.digital-out-34',
                            'PYM35' : 'motion.digital-out-35',
                            'PYM36' : 'motion.digital-out-36',
                            'PYM37' : 'motion.digital-out-37',
                            'PYM38' : 'motion.digital-out-38',
                            'PYM39' : 'motion.digital-out-39',
                            'PYM40' : 'motion.digital-out-40',
                            'PYM41' : 'motion.digital-out-41',
                            'PYM42' : 'motion.digital-out-42',
                            'PYM43' : 'motion.digital-out-43',
                            'PYM44' : 'motion.digital-out-44',
                            'PYM45' : 'motion.digital-out-45',
                            'PYM46' : 'motion.digital-out-46',
                            'PYM47' : 'motion.digital-out-47',
                            'PYM48' : 'motion.digital-out-48',
                            'PYM49' : 'motion.digital-out-49',
                            'PYM50' : 'motion.digital-out-50',
                            'PYM51' : 'motion.digital-out-51',
                            'PYM52' : 'motion.digital-out-52',
                            'PYM53' : 'motion.digital-out-53',
                            'PYM54' : 'motion.digital-out-54',
                            'PYM55' : 'motion.digital-out-55',
                            'PYM56' : 'motion.digital-out-56',
                            'PYM57' : 'motion.digital-out-57',
                            'PYM58' : 'motion.digital-out-58',
                            'PYM59' : 'motion.digital-out-59',
                            'PYM60' : 'motion.digital-out-60',
                            'PYM61' : 'motion.digital-out-61',
                            'PYM62' : 'motion.digital-out-62',
                            'PYM63' : 'motion.digital-out-63',
                            'PYM64' : 'motion.digital-out-64',
                            }
        
        for name,value in self.py_out_pins.items():
            self.pin_out = self.hglib_pin(self.h.newpin(name, hal.HAL_BIT, hal.HAL_IN))
            if str(name) in self.py_out_pins:
                self.py_out_pins.update({str(name): self.pin_out})

    def load_user_tabs(self):
        self.user_tab_modules = {}
        self.user_tabs = {}
        self.iniciar_hilo()
        sidebar_loaded = False;
        user_tabs_paths = INIFILE.findall("DISPLAY", "USER_TABS_PATH")

        for user_tabs_path in user_tabs_paths:
            user_tabs_path = os.path.expanduser(user_tabs_path)
            user_tab_folders = os.listdir(user_tabs_path)
            for user_tab in user_tab_folders:
                if not os.path.isdir(os.path.join(user_tabs_path, user_tab)):
                    continue

                module_name = "user_tab." + os.path.basename(user_tabs_path) + "." + user_tabs_path
                spec = importlib.util.spec_from_file_location(module_name, os.path.join(os.path.dirname(user_tabs_path), user_tab, user_tab + ".py"))
                self.user_tab_modules[module_name] = importlib.util.module_from_spec(spec)
                sys.modules[module_name] = self.user_tab_modules[module_name]
                spec.loader.exec_module(self.user_tab_modules[module_name])
                self.user_tabs[module_name] = self.user_tab_modules[module_name].UserTab()
                if self.user_tabs[module_name].property("sidebar"):
                    if sidebar_loaded == False:
                        sidebar_loaded = True
                        self.user_tabs[module_name].setParent(self.sb_page_4)
                        self.user_sb_tab.setText(self.user_tabs[module_name].objectName().replace("_", " "))
                    else:
                        # can not load more than one sidebar widget
                        pass
                else:
                    self.tabWidget.addTab(self.user_tabs[module_name], self.user_tabs[module_name].objectName().replace("_", " "))

        if sidebar_loaded == False:
            self.user_sb_tab.hide()
            self.dro_tab.setStyleSheet(self.user_sb_tab.styleSheet())

    def on_feed_unit_per_minute_entry_textChanged(self, value):
        if value:
            self.feed_unit_per_minute = float(value)
        else:
            self.feed_unit_per_minute = 0.0

    def on_feed_unit_per_minute_entry_returnPressed(self):
        cmd = "G94 F{}".format(self.feed_unit_per_minute)
        issue_mdi(cmd)

    def on_feed_per_rev_entry_textChanged(self, value):
        if value:
            self.feed_per_rev = float(value)
        else:
            self.feed_per_rev = 0.0000

    def btn_out_pressed(self):
        print("el valor de la in del dict es: ",hal.get_value(self.py_mcodes_pins["PYM27"]))
        #print("el valor de la motion digin 27 es ta tan ta tan",hal.get_value("motion.digital-out-27"))
        #tengo que hacer que cambie el valor de la salida y si el valor del estado de la salida es true lo haga false y viceversa
        key_cmd = self.sender().objectName()
        #esto es el valor que trae de key Cmd_CNC_OK
        #esto es la variable que tengo que mirar FbkOut_CNC_OK
        key_fbk = key_cmd.replace("Cmd", "FbkOut")
        #print(key_cmd, key_fbk)
        if not hal.get_value("qtpyvcp.{}.on".format(key_fbk)):
            self.py_out_pins[key_cmd].set(True)
        else:
            self.py_out_pins[key_cmd].set(False)
    
    #check sensors neumatic cmd
    def wait_for_sen_flag(self, sen_key):
        #print('el pin es',(hal.get_value('FbkOut_CNC_OK')))
        sen_check = hal.get_value("qtpyvcp.{}.on".format(sen_key))
        start_time = datetime.datetime.now()
        while not sen_check:
            sen_check = hal.get_value("qtpyvcp.{}.on".format(sen_key))
            elapsed_time = (datetime.datetime.now() - start_time).total_seconds()
            if elapsed_time >= self.TIMEOUT_PNEUMATIC:
                return False
            time.sleep(self.wait_time)
        return True

    def on_feed_per_rev_entry_returnPressed(self):
        cmd = "G95 F{}".format(self.feed_per_rev)
        issue_mdi(cmd)

    def on_css_sword_entry_textChanged(self, value):
        if value:
            self.css_sword = float(value)
        else:
            self.css_sword = 0.0

    def on_css_sword_entry_returnPressed(self):
        cmd = "G96 S{}".format(self.css_sword)
        issue_mdi(cmd)

    def on_rpm_mode_entry_textChanged(self, value):
        if value:
            self.rpm_mode = float(value)
        else:
            self.rpm_mode = 0

    def on_rpm_mode_entry_returnPressed(self):
        cmd = "G97 S{}".format(self.rpm_mode)
        issue_mdi(cmd)

    def on_use_tcp_clicked(self):
        if self.use_tcp.isChecked():
            self.use_tcp_mode.setText('1')
        else:
            self.use_tcp_mode.setText('0')

    def on_run_from_line_Btn_clicked(self):
        try:
            lineNum = int(self.run_from_line_Num.text())
        except:
            return False

        actions.program_actions.run(lineNum)
    
    class MasterThread(QThread):
        finished_signal = pyqtSignal()

        def __init__(self, parent=None):
            super().__init__()
            self.main_window = parent

        def run(self):
            try:
                hola = 0
                
                self.msleep(3000)
                while True:
                    #inicio = time.perf_counter()
                    result = self.main_window.gama_changer_1()
                    #fin = time.perf_counter()
                    #print(f"⏱ Tiempo de ejecución: {fin - inicio:.4f} segundos")
                    ##print("ESTADO DE SALIDAAAAAAAAAAAAAA",hal.get_value('qtpyvcp.FbkOut_Vel_CambioGamas.on'))
                    #hola += 1
                    #print("se ejecuta main thread",hola)
                    if result:
                        dict_cmds, step = result
                        rutina_nombre = 'gama_changer_1'

                        if step not in self.main_window.active_threads[rutina_nombre]:
                            print(f"🚀 Ejecutando {rutina_nombre} - Step {step} en nuevo hilo")
                            self.main_window.active_threads[rutina_nombre].append(step)

                            routine_thread = self.main_window.RoutineRunnerThread(
                                rutina_nombre=rutina_nombre,
                                step=step,
                                dict_cmds=dict_cmds,
                                parent=self.main_window
                            )

                            # solo mensaje, ya no removemos desde acá
                            routine_thread.finished_signal.connect(lambda: print("🔔 Thread finalizado"))
                            routine_thread.start()
                    self.msleep(1000)

            except Exception as e:
                print("❌ Error en MasterThread:", e)

            self.finished_signal.emit()



    def iniciar_hilo(self):
        print('hilo principal iniciado')
        time.sleep(2)
        # Crear una instancia del hilo con referencia a la ventana principal
        self.mi_hilo = self.MasterThread(parent=self)
        self.mi_hilo.start()

    
    class RoutineRunnerThread(QThread):
        finished_signal = pyqtSignal()

        def __init__(self, rutina_nombre, step, dict_cmds, parent=None):
            super().__init__()
            self.rutina_nombre = rutina_nombre
            self.step = step
            self.dict_cmds = dict_cmds
            self.main_window = parent

        def run(self):
            try:
                result = self.main_window.activar_salidas_con_check(self.dict_cmds, self.rutina_nombre)
                if result:
                    print(f"✅ Rutina {self.rutina_nombre} - Step {self.step} ejecutada correctamente")
                else:
                    print(f"❌ Rutina {self.rutina_nombre} - Step {self.step} falló")
            except Exception as e:
                print(f"❌ Excepción en {self.rutina_nombre} - Step {self.step}: {e}")

            # ✅ Eliminar el step dentro del hilo, siempre
            if self.step in self.main_window.active_threads[self.rutina_nombre]:
                self.main_window.active_threads[self.rutina_nombre].remove(self.step)
                print(f"🟣 Step {self.step} eliminado de active_threads de {self.rutina_nombre}")

            self.finished_signal.emit()




    def activar_salidas_con_check(self, dict_cmds):
        for key, value in dict_cmds.items():
            estado = value['estado']
            leyenda_error = value['leyenda_error']

            if not self.main_window.send_pneumatic(key, estado):
                print(leyenda_error)
                self.main_window.routine_error_messages[self.rutina_nombre].append(leyenda_error)
                self.main_window.err_routine = False
                return False

            time.sleep(0.3)

        return True




    def check_init(self, dict_init):
        s = linuxcnc.stat()
        s.poll()
        error_messages = []

        for key, value in dict_init.items():
            hal_name = key
            hal_value = hal.get_value(hal_name)
            estado_esperado = value['estado']
            leyenda_error = value['leyenda_error']

            if hal_value != estado_esperado:
                error_messages.append(leyenda_error)

        return error_messages


    def init_cycle(self):
        rutina_nombre = 'init_cycle'
        # STEP 0
        step = 0
        digin_init_step0 = {
            'qtpyvcp.FbkIn_Ind_Gama_I.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
            'qtpyvcp.FbkIn_Ind_Gama_II.on': {
                'leyenda_error': "SENSOR INDUCTIVO GAMA I",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_III.on': {
                'leyenda_error': "SENSOR INDUCTIVO GAMA I",
                'estado': 0
            },
        }

        errores = self.check_init(digin_init_step0)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 0")
            for err in errores:
                pass
                print("los checkinit por los cuales no ejecuta rutina son",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")
                self.actual_gear = 'gear1'

        # STEP 1
        step = 1
        digin_init_step1 = {
            'qtpyvcp.FbkIn_Ind_Gama_I.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_II.on': {
                'leyenda_error': "SENSOR INDUCTIVO GAMA I",
                'estado': 1
            },
            'qtpyvcp.FbkIn_Ind_Gama_III.on': {
                'leyenda_error': "SENSOR INDUCTIVO GAMA I",
                'estado': 0
            },
        }

        errores = self.check_init(digin_init_step1)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 1")
            for err in errores:
                pass
                print("los checkinit por los cuales no ejecuta rutina son",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")
                self.actual_gear = 'gear2'


        # STEP 2
        step = 2
        digin_init_step2 = {
            'qtpyvcp.FbkIn_Ind_Gama_I.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_II.on': {
                'leyenda_error': "SENSOR INDUCTIVO GAMA I",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_III.on': {
                'leyenda_error': "SENSOR INDUCTIVO GAMA I",
                'estado': 1
            },
        }

        errores = self.check_init(digin_init_step2)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 2")
            for err in errores:
                pass
                print("los checkinit por los cuales no ejecuta rutina son",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")
                self.actual_gear = 'gear3'

        return False

    def gama_changer_1(self):
        rutina_nombre = 'gama_changer_1'
        # STEP 0
        step = 0
        digin_init_step0 = {
            self.py_mcodes_pins["PYM18"]: {
                'leyenda_error': "m41 no está prendido",
                'estado': 1
            },
            'qtpyvcp.FbkIn_MotCab_Det.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_I.on': {
                'leyenda_error': "SENSOR INDUCTIVO GAMA I",
                'estado': 0
            }
        }

        errores = self.check_init(digin_init_step0)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            #print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 0")
            for err in errores:
                pass
                #print("los checkinit por ls cuales no ejecuta rutina son",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")
                dict_cmds = {
                    'Cmd_Vel_CambioGamas': {
                        'leyenda_error': "Step 0 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': True
                    },
                }
                return (dict_cmds, step)

        # STEP 1 (cuando ya se ejecutó step 0)
        step = 1
        digin_init_step1 = {
            self.py_mcodes_pins["PYM18"]: {
                'leyenda_error': "m41 no está prendido",
                'estado': 1
            },
            'qtpyvcp.FbkOut_Vel_CambioGamas.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
            'qtpyvcp.FbkIn_VelCambioGama.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
            'qtpyvcp.FbkIn_FrenoCab_Lib.on': {
                'leyenda_error': "SENSOR INDUCTIVO GAMA I",
                'estado': 1  # cambia la lógica para que valide el nuevo estado
            }
        }

        errores = self.check_init(digin_init_step1)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            #print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 1")
            for err in errores:
                pass
                #print("los checkinit por ls cuales no ejecuta rutina son 2",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")
                dict_cmds = {
                    'Cmd_EV_Gama_M41': {
                        'leyenda_error': "Step 1 - ACTIVA GAMA M43",
                        'estado': True
                    },
                    'Cmd_EV_Gamas_M42M43': {
                        'leyenda_error': "Step 1 - ACTIVA GAMA M43",
                        'estado': False
                    },
                    'Cmd_EV_Gama_M43': {
                        'leyenda_error': "Step 1 - ACTIVA GAMA M43",
                        'estado': True
                    },
                    'Cmd_EV_Gamas_M41M43': {
                        'leyenda_error': "Step 1 - ACTIVA GAMA M43",
                        'estado': False
                    },
                },
                return (dict_cmds, step)
        
        # STEP 2 (cuando ya se ejecutó step 1)
        step = 2
        digin_init_step2 = {
            self.py_mcodes_pins["PYM18"]: {
                'leyenda_error': "m41 no está prendido",
                'estado': 1
            },
            'qtpyvcp.FbkIn_Ind_Gama_I.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
            'qtpyvcp.FbkIn_Ind_Gama_II.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_III.on': {
                'leyenda_error': "SENSOR INDUCTIVO GAMA I",
                'estado': 0  # cambia la lógica para que valide el nuevo estado
            }
        }

        errores = self.check_init(digin_init_step2)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            #print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 2")
            for err in errores:
                pass
                #print("los checkinit por ls cuales no ejecuta rutina son 2",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")

                self.actual_gear = 'gear1'

                dict_cmds = {
                    self.py_mcodes_pins["PYM18"]: {
                        'leyenda_error': "Step 2 - BAJA M41",
                        'estado': 0
                    },
                    'Cmd_Vel_CambioGamas': {
                        'leyenda_error': "Step 2 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': False
                    },
                },

                return (dict_cmds, step)
        
        # STEP 3 (cuando ya se ejecutó step 1)
        step = 3
        digin_init_step3 = {
            self.py_mcodes_pins["PYM20"]: {
                'leyenda_error': "m41 no está prendido",
                'estado': 1
            },
            'qtpyvcp.FbkIn_MotCab_Det.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_II.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
        }

        errores = self.check_init(digin_init_step3)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            #print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 3")
            for err in errores:
                pass
                #print("los checkinit por ls cuales no ejecuta rutina son 3",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")

                dict_cmds = {
                    'Cmd_Vel_CambioGamas': {
                        'leyenda_error': "Step 3 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': True
                    }
                },

                return (dict_cmds, step)
            
        # STEP 4 (cuando ya se ejecutó step 1)
        step = 4
        digin_init_step4 = {
            self.py_mcodes_pins["PYM19"]: {
                'leyenda_error': "m41 no está prendido",
                'estado': 1
            },
            'qtpyvcp.FbkOut_Vel_CambioGamas.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
            'qtpyvcp.FbkIn_VelCambioGama.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
            'qtpyvcp.FbkIn_FrenoCab_Lib.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
        }

        errores = self.check_init(digin_init_step4)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            #print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 4")
            for err in errores:
                pass
                #print("los checkinit por ls cuales no ejecuta rutina son 4",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")

                dict_cmds = {
                    'Cmd_EV_Gama_M41': {
                        'leyenda_error': "Step 4 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': False
                    },
                    'Cmd_EV_Gamas_M42M43': {
                        'leyenda_error': "Step 4 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': True
                    },
                    'Cmd_EV_Gama_M43': {
                        'leyenda_error': "Step 4 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': True
                    },
                    'Cmd_EV_Gamas_M41M43': {
                        'leyenda_error': "Step 4 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': False
                    },
                },

                return (dict_cmds, step)
            
        # STEP 5 (cuando ya se ejecutó step 1)
        step = 5
        digin_init_step5 = {
            self.py_mcodes_pins["PYM19"]: {
                'leyenda_error': "m41 no está prendido",
                'estado': 1
            },
            'qtpyvcp.FbkIn_FbkIn_Ind_Gama_I.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_II.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
            'qtpyvcp.FbkIn_Ind_Gama_III.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
        }

        errores = self.check_init(digin_init_step5)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            #print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 5")
            for err in errores:
                pass
                #print("los checkinit por ls cuales no ejecuta rutina son 5",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")

                self.actual_gear = 'gear2'

                dict_cmds = {
                    self.py_mcodes_pins["PYM19"]: {
                        'leyenda_error': "Step 5 - BAJA M41",
                        'estado': 0
                    },
                    'Cmd_Vel_CambioGamas': {
                        'leyenda_error': "Step 5 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': False
                    },
                },

                return (dict_cmds, step)
            
        # STEP 6 (cuando ya se ejecutó step 1)
        step = 6
        digin_init_step6 = {
            self.py_mcodes_pins["PYM20"]: {
                'leyenda_error': "m41 no está prendido",
                'estado': 1
            },
            'qtpyvcp.FbkIn_Ind_Gama_I.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_III.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
        }

        errores = self.check_init(digin_init_step6)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            #print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 5")
            for err in errores:
                pass
                #print("los checkinit por ls cuales no ejecuta rutina son 5",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")

                self.actual_gear = 'gear2'

                dict_cmds = {
                    'Cmd_Vel_CambioGamas': {
                        'leyenda_error': "Step 6 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': False
                    },
                },

                return (dict_cmds, step)
            
        # STEP 7 (cuando ya se ejecutó step 1)
        step = 7
        digin_init_step7 = {
            self.py_mcodes_pins["PYM20"]: {
                'leyenda_error': "m41 no está prendido",
                'estado': 1
            },
            'qtpyvcp.FbkOut_Vel_CambioGamas.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
            'qtpyvcp.FbkIn_VelCambioGama.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
            'qtpyvcp.FbkIn_FrenoCab_Lib.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
        }

        errores = self.check_init(digin_init_step7)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            #print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 5")
            for err in errores:
                pass
                #print("los checkinit por ls cuales no ejecuta rutina son 5",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")

                self.actual_gear = 'gear2'

                dict_cmds = {
                    'Cmd_EV_Gama_M41': {
                        'leyenda_error': "Step 7 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': False
                    },
                    'Cmd_EV_Gamas_M42M43': {
                        'leyenda_error': "Step 7 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': False
                    },
                    'Cmd_EV_Gama_M43': {
                        'leyenda_error': "Step 7 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': False
                    },
                    'Cmd_EV_Gamas_M41M43': {
                        'leyenda_error': "Step 7 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': True
                    },
                },

                return (dict_cmds, step)
            
        # STEP 8 (cuando ya se ejecutó step 1)
        step = 8
        digin_init_step8 = {
            self.py_mcodes_pins["PYM20"]: {
                'leyenda_error': "m41 no está prendido",
                'estado': 1
            },
            'qtpyvcp.FbkIn_Ind_Gama_I.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_II.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 0
            },
            'qtpyvcp.FbkIn_Ind_Gama_III.on': {
                'leyenda_error': "MOTOR CABEZAL DETENIDO",
                'estado': 1
            },
        }

        errores = self.check_init(digin_init_step8)
        self.init_conditions_error_messages[rutina_nombre] = errores[:]

        if errores:
            #print(f"\n❌ Error en condiciones iniciales {rutina_nombre} - Step 8")
            for err in errores:
                pass
                #print("los checkinit por ls cuales no ejecuta rutina son 8",err)
        else:
            if step not in self.active_threads[rutina_nombre]:
                print(f"✅ Condiciones OK - {rutina_nombre} Step {step}")

                self.actual_gear = 'gear3'

                dict_cmds = {
                    self.py_mcodes_pins["PYM20"]: {
                        'leyenda_error': "Step 8 - BAJA M41",
                        'estado': 0
                    },
                    'Cmd_Vel_CambioGamas': {
                        'leyenda_error': "Step 8 - VELOCIDAD DE CAMBIO DE GAMAS",
                        'estado': False
                    },
                },

                return (dict_cmds, step)

        return False



    def check_init(self, dict_init):
        s = linuxcnc.stat()
        s.poll()
        error_messages = []

        for key, value in dict_init.items():
            #hal_name = f"qtpyvcp.{key}.on"
            hal_name = key
            hal_value = hal.get_value(hal_name)
            estado_esperado = value['estado']
            leyenda_error = value['leyenda_error']

            if hal_value != estado_esperado:
                # podés retornar una tupla (nombre, mensaje)
                error_messages.append((leyenda_error))
        #print(error_messages)
        return error_messages
    

    #send neumatic commands
    def send_pneumatic(self, key, bool_value, second_key=None, second_bool_value=None):
        try:
            self.py_out_pins[key].set(bool_value)
            time.sleep(0.1)
            if second_key:
                self.py_out_pins[second_key].set(second_bool_value)
            #print 'comando enviado exitoso'
            return True
        except:
            return False

    @Slot(QAbstractButton)
    def on_sidebartabGroup_buttonClicked(self, button):
        self.sidebar_widget.setCurrentIndex(button.property('page'))

    # MDI Panel
    @Slot(QAbstractButton)
    def on_gcodemdibtnGroup_buttonClicked(self, button):
        self.gcode_mdi.setCurrentIndex(button.property('page'))

    @Slot(QAbstractButton)
    def on_btngrpMdi_buttonClicked(self, button):
        char = str(button.text())
        text = self.mdiEntry.text() or 'null'
        if text != 'null':
            text += char
        else:
            text = char
        self.mdiEntry.setText(text)

    def mdiBackSpace_clicked(parent):
        if len(parent.mdiEntry.text()) > 0:
            text = parent.mdiEntry.text()[:-1]
            parent.mdiEntry.setText(text)

    def mdiSpace_clicked(parent):
        text = parent.mdiEntry.text() or 'null'
        # if no text then do not add a space
        if text != 'null':
            text += ' '
            parent.mdiEntry.setText(text)

    @Slot(QAbstractButton)
    def on_spindlerpmsourcebtnGroup_buttonClicked(self, button):
        self.spindle_rpm_source_widget.setCurrentIndex(button.property('page'))





