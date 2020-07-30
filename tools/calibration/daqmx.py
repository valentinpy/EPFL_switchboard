# -*- coding: utf-8 -*-
"""
Classes pour simplifier l'acquisition NI
Measurement() : permet l'acquisition et l'emission de signaux sur une carte NI
Usage :
    #Mesure analogique simple
        mesureAnalogique=mesure()
        sampling_frequency=1000 #Fréquence d'echantillonnage (Hz)
        t=1 #Temps d'acquisition (s)
        sample_number=t*sampling_frequency #Nombre d'echantillons par voie
        (t,data)=mesureAnalogique.set_analog_measurement(['/Dev1/Ai0','/Dev1/Ai1'],sampling_frequency,sample_number)
        #Mesure 1 seconde à 1kHz
    #Mesure analogique continue
        mesureAnalogique.set_analog_measurement(['/Dev1/Ai0','/Dev1/Ai1'],sampling_frequency,typeEch='continu')
        initialTime=time.clock()
        tdata=np.empty((2,0))
        data=np.empty((2,0))
        while(time.clock()-initialTime<t):
            (tempTdata,tempdata)=mesureAnalogique.get_buffer()
            tdata=np.append(tdata,tempTdata)
            data=np.append(data,tempdata)

"""
import PyDAQmx as pDaq
import threading
import ctypes
import numpy as np
import scipy.signal
import ast


class DaqMx():
    """Classe pour simplifier les acquisitions NI DaqMx"""

    def __init__(self):
        self.continuous_acquisition_flag = False
        self.acquisition_thread = None
        self.daqLock = threading.Lock()
        # Verrou pour le buffer partagé de l'acquisition continue
        self.continuous_acquisition_lock = threading.Lock()
        self.timer_continuous_acquisition = 0.05
        self.averaging_index = 1
        self.signals_sum = None

    def __del__(self):
        if self.acquisition_thread:
            self.acquisition_thread.cancel()
        self.continuous_acquisition_flag = False

    # Fonctions générales
    def get_devices(self):
        """Returns the list of available devices (useful for listing available channels)"""
        buffer_size = 1000
        the_buffer = ctypes.create_string_buffer(buffer_size)
        pDaq.DAQmxGetSysDevNames(the_buffer, buffer_size)
        devices = str(the_buffer.value.decode()).split(",")
        return devices

    def get_max_sampling_rate(self, task):
        """Returns maximum sampling rate for a given task"""
        data = pDaq.float64()
        task.GetSampClkMaxRate(pDaq.byref(data))
        return data.value

    def get_channels(self, task):
        """Recuperer les channels de la tache"""
        s = ctypes.create_string_buffer(1000)
        task.GetTaskChannels(s, 1000)
        channels = s.value.decode()
        channels = [str.strip(c) for c in channels.split(',')]
        return channels

    def get_task_sampling_rate(self, task):
        """Recuperer la fréquence d'echantillonnage de la tache"""
        data = ctypes.c_double()
        task.GetSampClkRate(pDaq.byref(data))
        return data.value

    def get_task_samples_number(self, task):
        """Recuperer le nombre d'echantillon par voie de la tache"""
        data = ctypes.c_ulonglong()
        task.GetSampQuantSampPerChan(pDaq.byref(data))
        return data.value

    def deinterleave_signals(self, sig, n):
        """Déentrelacement des signaux pour les acquisitions multivoies"""
        return np.transpose(np.reshape(sig, (int(len(sig) / n), int(n))))

    def interleave_signals(self, sigs):
        """Entrelacer les signaux pour la sortie simultanée analogique"""
        shape = np.shape(sigs)
        nSig = len(sigs)
        interleavedData = np.empty(shape[0] * shape[1], dtype=np.float64)
        for i, sig in enumerate(sigs):
            interleavedData[i::nSig] = sig
        return interleavedData

    def align_waveform(self, t, sig):
        """Aligner les signaux par interpolation linéaire (alignement sur le vecteur t[0])
        Utile pour réaliser la fonction de réponse en fréquence par FFT"""       
        for i in range(1, len(sig)):
            sig[i] = np.interp(t[0], t[i], sig[i])
        return sig

    def shift_waveform(self, t, sig):
        """Décale les signaux d'un echantillon par interpolation pour la sortie analogique"""
        sampling_frequency = 1.0 / (t[1] - t[0])
        for i in range(1, np.shape(sig)[0]):
            nt = t + i * 1.0 / sampling_frequency * np.ones(np.shape(t))
            sig[i] = np.interp(nt, t, sig[i])
        return sig

    def resample_signal(self, sig, oldFreq, newFreq):
        """Resample signal to a new sampling frequency, keeping the duration of the signal"""
        tmax = len(sig) / oldFreq
        nt = np.linspace(0, tmax, newFreq * tmax)
        t = np.linspace(0, tmax, oldFreq * tmax)
        # On redefinit le signal pour qu'il corresponde à la nouvelle fréquence
        nsig = np.interp(nt, t, sig)
        return nsig

    def get_terminal_name_with_prefix(self, task, terminalName):
        """Retourne le nom du terminal avec le prefixe necessaire
        (utile pour ai/SampleClock par example)"""
        numDevices = pDaq.uInt32()
        productCategory = pDaq.int32()
        device = pDaq.create_string_buffer(256)
        task.GetTaskNumDevices(pDaq.byref(numDevices))
        numDevices = numDevices.value
        i = 1
        triggerName = ''
        while i <= numDevices:
            task.GetNthTaskDevice(i, device, 256)
            i += 1
            pDaq.DAQmxGetDevProductCategory(
                device.value, pDaq.byref(productCategory))
            if productCategory.value != pDaq.DAQmx_Val_CSeriesModule and \
                    productCategory.value != pDaq.DAQmx_Val_SCXIModule:
                triggerName += '/'
                triggerName += device.value.decode() + '/' + terminalName
                break
        return triggerName
############################################################################
    # Acquisition Analogique

    def get_available_analog_input(self):
        """Returns available AI channels"""
        buffer_size = 1000
        the_buffer = pDaq.create_string_buffer(buffer_size)
        devices = self.get_devices()
        AI = []
        for dev in devices:
            if dev != '':
                pDaq.DAQmxGetDevAIPhysicalChans(dev, the_buffer, buffer_size)
                if the_buffer.value != '':
                    AI = AI + [str.strip(c)
                               for c in the_buffer.value.decode().split(',')]
        return AI

    def get_max_analog_input_rate(self, voie=None):
        """Returns AI Maximum sampling frequency for a given channel or multiple channels"""
        task = pDaq.Task()
        if not voie:
            voie = self.get_available_analog_input()[0]
        task.CreateAIVoltageChan(
            voie, "", pDaq.DAQmx_Val_RSE, -10.0, 10.0, pDaq.DAQmx_Val_Volts, None)
        maxSampl = self.get_max_sampling_rate(task)
        del task
        return maxSampl

    def set_analog_measurement(self, channels, sampling_frequency, sample_number=100, Vmin=-10.0, Vmax=10.0, terminal_configuration='RSE', typeEch='fini', trigger=None, voieTrigger=None, triggerLevel=0, triggerSlope='montant', timeout=10.0):
        """
        Initialise une acquisition Analogique
        channels : La ou les voies d'acquisition
        sampling_frequency : Fréquence d'acquisition (par voie)
        sample_number : Nombre d'echantillons par voie (inutile en mode continu)
        Vmin : Tension minimale mesurée (défaut : -10)
        Vmax : Tension maximale mesurée (défaut : 10)
        terminal_configuration : 'RSE' (défaut): Mesure par rapport à la masse, 'NRSE' : Mesure par rapport à AISENSE, 'Diff' : Mesure Différentielle
        typeEch : 'fini' (défaut) : acquérit un certain nombre d'échantillon et arrete la tâche. 'continu' : acquérit jusqu'à appel de stop_continuous_acquisition
        trigger : 
            None (défaut) : déclenchement de l'acquisition dès le lancement de la tâche, 
            'Analog' : déclenchement lorsque la valeur de la voie voieTrigger dépasse (ou est en-dessous en fonction de triggerSlope) la valeur triggerLevel, 
            'Digital' : déclenchement sur trigger numérique sur la voie voieTrigger et le front défini par triggerSlope
        voieTrigger : Voie trigger
        triggerLevel : Niveau trigger (Analog seulement)
        triggerSlope : 'Montant' (défaut) ou 'Descendant'
        timeout : temps maximum d'attente pour l'acquisition finie en secondes (défaut : 10s)
        """

        # Si l'argument n'est pas une liste (une seule channel par exemple)
        if not isinstance(channels, list):
            channels = [channels]  # On le converti en liste
        self.acquisition_task = pDaq.Task()  # Initialisation de la tache DaqMx
        self.daqLock.acquire()  # Verrou daqMx pour le multithreading
        self.acquisitionBuffer = np.zeros(
            (len(channels), 0))  # Initialisation des buffers
        self.acquisitionBufferTime = np.zeros((len(channels), 0))  # de données
        self.acquisition_task.CreateAIVoltageChan(', '.join(channels), "", self._get_terminal_config_constant(
            terminal_configuration), float(Vmin), float(Vmax), pDaq.DAQmx_Val_Volts, None)  # On crée la voie d'acquisition
        # On récupère la fréquence max dont la carte est capable
        maximum_sampling_rate = self.get_max_sampling_rate(self.acquisition_task)
        if(sampling_frequency > maximum_sampling_rate):  # Si la fréquence spécifiée est trop haute
            t = float(sample_number) / sampling_frequency  # On ajuste le nombre d'echantillons pour
            new_sample_number = maximum_sampling_rate * t  # que le temps d'acquisition reste constant
            sample_number = new_sample_number  # tout en utilisant la frequence
            sampling_frequency = maximum_sampling_rate  # maximale d'echantillonnage
            print(u"Fréquence d'échantillonnage d'entrée trop élevée, utilisation de la valeur max : %.1f" % maximum_sampling_rate)
            print(
                u"Ajustement du nombre d'échantillon pour un temps d'acquisition constant")
        sample_number = int(sample_number)
        if(typeEch == 'fini'):  # Si l'acquisision est une acquisition finie
            self.continuous_acquisition_flag = False  # On met le flag à 0 et on configure l'horloge
            self.acquisition_task.CfgSampClkTiming(
                "", sampling_frequency, pDaq.DAQmx_Val_Rising, pDaq.DAQmx_Val_FiniteSamps, sample_number * len(channels))
        elif(typeEch == 'continu'):  # Si elle est continue
            self.continuous_acquisition_flag = True  # Le flag est à 1
            # Le nombre d'echantillons lus au max est donné par 2*periode de
            # lecture du buffer*sampling_frequency
            sample_number = int(self.timer_continuous_acquisition * sampling_frequency * 5)
            # Configuration de l'horloge
            self.acquisition_task.CfgSampClkTiming(
                "", sampling_frequency, pDaq.DAQmx_Val_Rising, pDaq.DAQmx_Val_ContSamps, sample_number * len(channels))
        if(trigger == 'analog'):  # Si on configure un trigger analogique
            trigger_edge = pDaq.DAQmx_Val_RisingSlope
            if(triggerSlope == 'descendant'):
                trigger_edge = pDaq.DAQmx_Val_FallingSlope
            self.acquisition_task.CfgAnlgEdgeStartTrig(
                voieTrigger, trigger_edge, triggerLevel)
        elif(trigger == 'digital'):  # Ou digital
            trigger_edge = pDaq.DAQmx_Val_Rising
            if(triggerSlope == 'descendant'):
                trigger_edge = pDaq.DAQmx_Val_Falling
            self.acquisition_task.CfgDigEdgeStartTrig(voieTrigger, trigger_edge)
        self.acquisition_task.StartTask()  # Démarrage de l'acquisition
        self.daqLock.release()  # Relachement du verrou pour le multitask
        return self._read_analog_buffer(timeout)  # Début de la lecture carte

    def _get_terminal_config_constant(self, terminal_configuration='RSE'):
        if(terminal_configuration == 'RSE'):
            config_constant = pDaq.DAQmx_Val_RSE
        elif(terminal_configuration == 'NRSE'):
            config_constant = pDaq.DAQmx_Val_NRSE
        elif(terminal_configuration == 'Diff'):
            config_constant = pDaq.DAQmx_Val_Diff
        elif(terminal_configuration == 'PseudoDiff'):
            config_constant = pDaq.DAQmx_Val_PseudoDiff
        else:
            config_constant = pDaq.DAQmx_Val_Cfg_Default
        return config_constant

    def set_timer_continuous_acquisition(self, val):
        """Défini la période à laquelle la carte est lue pour l'acquisition continue en seconde (valeur par défaut : 0.05)"""
        self.timer_continuous_acquisition = val

    def _read_analog_buffer(self, timeout=10.0):
        """Lecture des voies analogiques"""
        time_ai = None  # Initialisation des variables de données
        data_ai = None
        read = pDaq.int32()  # Constante entier pour le nombre d'echantillon lus

        self.daqLock.acquire()  # On verrouille l'acquisition quand c'est disponible
        # On récupére les voies d'acquisition
        AI = self.get_channels(self.acquisition_task)
        # et la fréquence d'echantillonnage
        sampling_frequency = self.get_task_sampling_rate(self.acquisition_task)
        channel_number = len(AI)  # Nombre de voies
        if(self.continuous_acquisition_flag):
            # On récupère le nombre d'échantillons par voie lus (au max)
            sample_number = self.get_task_samples_number(self.acquisition_task)
            # On crée un buffer de donnée de la taille correspondante
            data = np.zeros((sample_number * channel_number,), dtype=np.float64)
            self.acquisition_task.ReadAnalogF64(-1, 0, pDaq.DAQmx_Val_GroupByScanNumber, data,
                                       sample_number * channel_number, pDaq.byref(read), None)  # On lit le buffer de mesure
        else:
            # On récupère le nombre d'échantillons par voie lus (au max)
            sample_number = self.get_task_samples_number(self.acquisition_task)
            # On crée un buffer de donnée de la taille correspondante
            data = np.zeros((sample_number * channel_number,), dtype=np.float64)
            self.acquisition_task.ReadAnalogF64(sample_number, timeout, pDaq.DAQmx_Val_GroupByScanNumber,
                                       data, sample_number * channel_number, pDaq.byref(read), None)  # On lit le buffer de mesure
        self.daqLock.release()  # On relache le verrou DaqMx

        sample_numberRead = read.value  # On récupere le nombre d'echantillons lus
        if(sample_numberRead > 0):  # Si il y a effectivement des echantillons lus
            # On crée le buffer de temps
            t = np.linspace(0, float(sample_numberRead) / sampling_frequency, sample_numberRead * channel_number)
            # On réduit le buffer de données à sa longueur effective lue
            data = data[:sample_numberRead * channel_number]
            # On désentrelace les différentes voies pour obtenir les temps
            # correspondant
            time_ai = self.deinterleave_signals(t, channel_number)
            # On désentrelace les différentes voies pour obtenir les data
            # correspondant
            data_ai = self.deinterleave_signals(data, channel_number)

            if(self.continuous_acquisition_flag):  # Si l'acquisition est continue
                self.continuous_acquisition_lock.acquire()  # On récupère le verrou d'acquisition continue
                self.acquisitionBuffer = np.append(
                    self.acquisitionBuffer, data_ai, axis=1)  # On ajoute les data au buffer
                # Si la longueur des temps est nulle
                if(np.shape(self.acquisitionBufferTime)[1] == 0):
                    # Alors on ajoute les données de temps en brut
                    self.acquisitionBufferTime = np.append(
                        self.acquisitionBufferTime, time_ai, axis=1)
                else:  # Sinon on prend le dernier temps et on lui ajoute les valeurs de temps obtenues
                    self.acquisitionBufferTime = np.append(
                        self.acquisitionBufferTime, time_ai + np.flipud(self.acquisitionBufferTime[:, -1:]), axis=1)
                # On relache le verrou quand les données sont écrites
                self.continuous_acquisition_lock.release()

            else:  # Si c'est une acquisition finie, le buffer contient directement les données lues
                self.acquisitionBufferTime = time_ai
                self.acquisitionBuffer = data_ai

        if(self.continuous_acquisition_flag):  # Si l'acquisition est continue
            # On reprogramme un cyle de lecture pour un temps égal à 80% de la
            # période de remplissage du buffer
            self.acquisition_thread = threading.Timer(
                self.timer_continuous_acquisition, self._read_analog_buffer)
            self.acquisition_thread.start()  # Et le thread est lancé
        if(time_ai is not None):
            if(np.shape(time_ai)[0] == 1):
                time_ai = time_ai[0]
                data_ai = data_ai[0]
        # On retourne les données lues (utile pour l'acquisition finie)
        return (time_ai, data_ai)

    def stop_continuous_acquisition(self):
        """Stopper l'acquisition continue"""
        self.continuous_acquisition_flag = False  # Le Flag est arreté
        if(self.acquisition_thread):  # Si le thread d'acquisition existe
            if(self.acquisition_thread.is_alive()):  # Et il fonctionne
                self.acquisition_thread.join()  # On attend la fin
            else:
                self.acquisition_thread.cancel()  # Ou on l'annule
        self.acquisition_task.StopTask()  # Et on arrete la tache d'acquisition analogique

    def get_buffer(self):
        """Lire le buffer de données de l'acquisition continue et vide le buffer en conservant la derniere valeur"""
        self.continuous_acquisition_lock.acquire()  # On récupère le verrou de l'acquisition continu
        # Initialisation des buffers de données
        buff = np.empty((np.shape(self.acquisitionBuffer)[0], 0))
        buffT = np.empty((np.shape(self.acquisitionBuffer)[0], 0))
        if(self.continuous_acquisition_flag):  # Si on est en acquisition continue
            if(self.acquisition_thread):  # Et que le thread existe bien
                # Si il reste plus d'une donnée dans le buffer
                if(np.shape(self.acquisitionBuffer)[1] > 1):
                    # On charge les données dans les buffers
                    buff = self.acquisitionBuffer[:, 0:-1]
                    # Sans la dernière (pour historique de temps)
                    buffT = self.acquisitionBufferTime[:, 0:-1]
                    self.acquisitionBuffer = self.acquisitionBuffer[
                        :, -1:]  # Et on vide les buffers
                    self.acquisitionBufferTime = self.acquisitionBufferTime[
                        :, -1:]  # en laissant la derniere
        else:  # Si l'acquisition est finie ou a été arretée
            buff = self.acquisitionBuffer  # On retourne toute les données qu'il y a
            buffT = self.acquisitionBufferTime
        self.continuous_acquisition_lock.release()  # On relache le verrou ensuite
        return (buffT, buff)  # On retourne les données du buffer

##########################################################################
    # Sorties Analogiques
    def getAO(self):
        """Returns available AO channels"""
        buffer_size = 1000
        the_buffer = pDaq.create_string_buffer(buffer_size)
        devices = self.get_devices()
        AO = []
        for dev in devices:
            if dev != '':
                pDaq.DAQmxGetDevAOPhysicalChans(dev, the_buffer, buffer_size)
                if(the_buffer.value != ''):
                    AO = AO + [str.strip(c)
                               for c in the_buffer.value.decode().split(',')]
        return AO

    def getMaxAOSampling(self, voie=None):
        """Returns AO Maximum sampling frequency for a given channel or multiple channels"""
        task = pDaq.Task()
        if(not(voie)):
            voie = self.getAO()[0]
        task.CreateAOVoltageChan(
            voie, "", -10.0, 10.0, pDaq.DAQmx_Val_Volts, None)
        maxSampl = self.get_max_sampling_rate(task)
        del(task)
        return maxSampl

    def signalSine(self, t=0, freq=1000, ampl=1, off=0, phase=0):
        """Génération d'un signal sinus"""
        sine = (np.float64)(ampl * np.sin(2 * np.pi * freq * t + phase) + off)
        return sine

    def signalTriangle(self, t=0, freq=1000, ampl=1, off=0, phase=0):
        """Génération d'un signal triangle"""
        return self.signalSawtooth(t, freq, ampl, off, phase, 0.5)

    def signalPWM(self, t=0, freq=1000, ampl=1, off=0, phase=0, duty=0.5):
        """Génération d'un signal PWM"""
        PWM = ampl * \
            scipy.signal.square(2 * np.pi * freq * t + phase, duty) + off
        return PWM

    def signalSquare(self, t=0, freq=1000, ampl=1, off=0, phase=0):
        """Génération d'un signal Carré"""
        return self.signalPWM(t, freq, ampl, off, phase, 0.5)

    def signalSawtooth(self, t=0, freq=1000, ampl=1, off=0, phase=0, width=1):
        """Génération d'un signal en dent de scie"""
        trig = ampl * \
            scipy.signal.sawtooth(2 * np.pi * freq * t + phase, width) + off
        return trig

    def signalSweep(self, fstart=1000, fstop=100000, timeSweep=1, amplitude=1, sampling_frequency=1000000, typeSweep='Log'):
        t = np.linspace(0, timeSweep, sampling_frequency * timeSweep)
        if(typeSweep == 'Lin'):
            # Lin chirp
            sig = (np.float64)(amplitude * scipy.signal.chirp(t,
                                                              fstart, timeSweep, fstop, 'linear'))
        elif(typeSweep == 'Log'):
            # Log chirp
            sig = (np.float64)(amplitude * scipy.signal.chirp(t,
                                                              fstart, timeSweep, fstop, 'logarithmic'))
        return (t, sig)

    def generateurDeFonction(self, channels=None, freq=1000, ampl=1, off=0, phase=0, typeSignal=0, nperiod=10, sample_numberPerPeriod='max'):
        """Simple function generator output
        Possible to pass one or multiple channels as arrays
        typeSignal: 0=sine, 1=triangle, 2=square"""
        if not(isinstance(channels, np.ndarray) or isinstance(channels, list)):
            channels = np.array(channels)
        channel_numbernels = len(channels)

        def extentToArray(valueToExtend, n):
            if not(isinstance(valueToExtend, np.ndarray) or isinstance(valueToExtend, list)):
                valueToExtend = valueToExtend * np.ones(n)
            elif(len(valueToExtend) < n):
                valueToExtend = np.append(valueToExtend, np.ones(
                    n - len(valueToExtend)) * valueToExtend[-1])
            else:
                valueToExtend = [float(c) for c in valueToExtend]
            return valueToExtend
        freq = extentToArray(freq, channel_numbernels)
        ampl = extentToArray(ampl, channel_numbernels)
        off = extentToArray(off, channel_numbernels)
        phase = extentToArray(phase, channel_numbernels)
        phase = np.array(phase) * np.pi / 180
        typeSignal = extentToArray(typeSignal, channel_numbernels)
        t = np.linspace(0, nperiod / np.min(freq), nperiod * 20)
        sigs = np.empty((channel_numbernels, np.shape(t)[0]))
        for i in range(0, channel_numbernels):
            if(typeSignal[i] == 0):
                sigs[i] = self.signalSine(
                    t, freq[i], ampl[i], off[i], phase[i])
            elif(typeSignal[i] == 1):
                sigs[i] = self.signalTriangle(
                    t, freq[i], ampl[i], off[i], phase[i])
            elif(typeSignal[i] == 2):
                sigs[i] = self.signalSquare(
                    t, freq[i], ampl[i], off[i], phase[i])
            elif(typeSignal[i] == 3):
                sigs[i] = self.signalSawtooth(
                    t, freq[i], ampl[i], off[i], phase[i])
            elif(typeSignal[i] == 4):
                sigs[i] = self.signalSawtooth(
                    t, freq[i], ampl[i], off[i], phase[i], width=0)
        self.sortieAnalogique(channels, sigs, t, resample=sample_numberPerPeriod)
        return (t, sigs)

    def sortieAnalogique(self, channels, signals, t=None, interpoler=True, typeEch='continu', resample=None, trigger=None, voieTrigger=None, triggerLevel=None, triggerSlope='montant', timeout=10.0):
        """
        sortieAnalogique permet de générer une sortie analogique sur une carte NI        
        channels : liste ([]) des channels sur lesquels générer le signal
        signals : signaux à genérer signals[0] correspond à la voie 0 signals[1] à la voie 1...
        t : vecteur de temps correspondant au signals[0]
        interpoler : interpole les signaux afin de les faire correspondre au décalage d'un échantillon du au multiplexage 
        typeEch : type d'échantillonnage 'continu' par défaut, peut-être 'fini'
        resample : None, pas de réchantillonnage, 'max' : reechantillonage à la valeur max possible de la carte.
        trigger : None, 'analog' : trigger sur voie analogique, 'digital' : trigger sur voie digital, 'analogSync' : trigger sur horloge d'echantillonage d'acquisition analogique (pour la synchro AI/AO)
        voieTrigger : voie de trigger, dans un cas AnalogSync créer l'emission avant l'acquisition !
        triggerLevel : Niveau de déclenchement (uniquement pour trigger analogique)
        triggerSlope : type de front pour le trigger ('montant' ou 'descendant')
        timeout : temps d'attente avant d'abandonner la tâche (en cas de non déclenchement)        
        """
        if not isinstance(channels, list):
            channels = [channels]
        if(len(np.shape(signals)) == 1):
            signals = np.array([signals])
        # Si le vecteur de signal ne contient qu'un élément par voie
        if(np.shape(signals)[1] < 2):
            # On met deux élément pour le bon fonctionnement de la fonction
            signals = np.transpose(
                np.resize(np.transpose(signals), (2, np.shape(signals)[0])))
        if(t is None):
            t = np.array([0, 1.0 / 100])
        self.daqLock.acquire()
        self.taskEm = pDaq.Task()
        self.taskEm.CreateAOVoltageChan(
            ','.join(channels), "", -10, 10, pDaq.DAQmx_Val_Volts, None)
        sampling_frequency = 1.0 / (t[1] - t[0])
        maximum_sampling_rateO = self.get_max_sampling_rate(self.taskEm)
        if(sampling_frequency > maximum_sampling_rateO or resample == 'max'):
            resampledSig = []
            for sig in signals:
                # On redefinit le signal pour qu'il corresponde à la nouvelle
                # fréquence
                resampledSig.append(self.resample_signal(sig, sampling_frequency, maximum_sampling_rateO))
            sampling_frequency = maximum_sampling_rateO
            signals = np.array(resampledSig)
            t = np.linspace(np.min(t), np.max(t), np.shape(signals)[1])
            print(u"Fréquence d'échantillonnage de sortie trop élevée, utilisation de la valeur max : %.1f" % maximum_sampling_rateO)
        if(np.shape(signals)[0] > 1):
            if(interpoler):
                signals = self.shift_waveform(t, signals)
        sample_numberPerChan = int(np.shape(signals)[1])

        signals = self.interleave_signals(signals)
        if(typeEch == 'continu'):
            self.taskEm.CfgSampClkTiming(
                "", sampling_frequency, pDaq.DAQmx_Val_Rising, pDaq.DAQmx_Val_ContSamps, sample_numberPerChan)
        elif(typeEch == 'fini'):
            self.taskEm.CfgSampClkTiming(
                "", sampling_frequency, pDaq.DAQmx_Val_Rising, pDaq.DAQmx_Val_FiniteSamps, sample_numberPerChan)
        if(trigger == 'analog'):
            trigger_edge = pDaq.DAQmx_Val_RisingSlope
            if(triggerSlope == 'descendant'):
                trigger_edge = pDaq.DAQmx_Val_FallingSlope
            self.taskEm.CfgAnlgEdgeStartTrig(
                voieTrigger, trigger_edge, triggerLevel)
        elif(trigger == 'digital'):
            trigger_edge = pDaq.DAQmx_Val_Rising
            if(triggerSlope == 'descendant'):
                trigger_edge = pDaq.DAQmx_Val_Falling
            self.taskEm.CfgDigEdgeStartTrig(voieTrigger, trigger_edge)
        elif(trigger == 'analogSync'):
            self.daqLock.release()
            self.set_analog_measurement(voieTrigger, 100, 2)
            self.daqLock.acquire()
            trigName = self.get_terminal_name_with_prefix(
                self.acquisition_task, 'ai/StartTrigger')
            self.taskEm.CfgDigEdgeStartTrig(trigName, pDaq.DAQmx_Val_Rising)
        self.taskEm.WriteAnalogF64(
            sample_numberPerChan, False, 10, pDaq.DAQmx_Val_GroupByScanNumber, signals, None, None)
        self.taskEm.StartTask()
        self.daqLock.release()
        if(t is not None):
            t = np.linspace(np.min(t), np.max(t), np.shape(signals)[0])
        return (t, signals)

    def stopOutput(self):
        """Stop current output task"""
        voies=self.get_channels(self.taskEm)
        self.taskEm.StopTask()
        self.sortieAnalogique(voies,np.zeros(len(voies)))
        self.taskEm.StopTask()

    def configureAITriggeredOutput(self, voieOut, voieIn, sig, sampling_frequency):
        """Configure output channel with starting trigger associated to the Analog Input Trigger
        Allows to perform synch output/input task"""
        if(len(np.shape(sig)) == 1):
            sample_number = np.shape(sig)[0]
        else:
            sample_number = np.shape(sig)[1]
        t = np.linspace(0, float(sample_number) / sampling_frequency, sample_number)
        self.sortieAnalogique(
            voieOut, sig, t, trigger='analogSync', voieTrigger=voieIn)

# Entrees numériques

    # Compteurs
    def getCountersChannels(self):
        """Recupere les voies de compteur disponibles"""
        buffer_size = 1000
        the_buffer = ctypes.create_string_buffer(buffer_size)
        devices = self.get_devices()
        self.counters = []
        for dev in devices:
            if dev != '':
                pDaq.DAQmxGetDevCIPhysicalChans(dev, the_buffer, buffer_size)
                if(the_buffer.value != ''):
                    self.counters = self.counters + \
                        [str.strip(c) for c in the_buffer.value.decode().split(',')]
        return self.counters

    def acquisitionEncodeur(self, voie, echSrc, pulsesPerRev=500, decodingType='X4', enableIndex=False, indexValue=0.0, indexPhase='AHighBHigh', units='deg', initialAngle=0):
        pass

    def initEncoderWithAnalogSamplingClock(self, voie, Fech, sample_number=100, pulsesPerRev=500, decodingType='X4', enableIndex=False, indexValue=0.0, indexPhase='AHighBHigh', units='deg', initialAngle=0):
        """Initialisation d'une tache de capture encodeur"""
        self.taskCounter = pDaq.Task()
        if(indexPhase == 'AHighBHigh'):
            indexPhase = pDaq.DAQmx_Val_AHighBHigh
        elif(indexPhase == 'AHighBLow'):
            indexPhase = pDaq.DAQmx_Val_AHighBLow
        elif(indexPhase == 'ALowBHigh'):
            indexPhase = pDaq.DAQmx_Val_ALowBHigh
        elif(indexPhase == 'ALowBLow'):
            indexPhase = pDaq.DAQmx_Val_ALowBLow
        if(units == 'deg'):
            units = pDaq.DAQmx_Val_Degrees
        else:
            units = pDaq.DAQmx_Val_Radians
        if(decodingType == 'X4'):
            decodingType = pDaq.DAQmx_Val_X4
        elif(decodingType == 'X1'):
            decodingType = pDaq.DAQmx_Val_X1
        elif(decodingType == 'X2'):
            decodingType = pDaq.DAQmx_Val_X2
        elif(decodingType == 'TwoPulseCounting'):
            decodingType = pDaq.DAQmx_Val_TwoPulseCounting

        self.taskCounter.CreateCIAngEncoderChan(voie, "", decodingType, enableIndex, float(
            indexValue), indexPhase, units, pulsesPerRev, float(initialAngle), None)
        # On définit une horloge de cadencement basée sur l'acquisition
        # analogique
        sampleClock = '/' + self.getDeviceName(voie) + '/ai/SampleClock'
        self.taskCounter.CfgSampClkTiming(
            sampleClock, Fech, pDaq.DAQmx_Val_Rising, pDaq.DAQmx_Val_ContSamps, sample_number)
        self.taskCounter.StartTask()
        self.encoderAcquisition = True
        self.encoderTimeBuffer = np.array([])
        self.encoderPosBuffer = np.array([])
        self.encoderSpeedBuffer = np.array([])
        self.encoderAccelerationBuffer = np.array([])
        self.readTimer()

    def readTimer(self):
        if(self.taskCounter):
            # La fréquence d'echantillonnage
            sampling_frequency = self.get_task_sampling_rate(self.taskCounter)
            # Le nombre d'échantillons par voie
            sample_number = self.get_task_samples_number(self.taskCounter)
            data = np.zeros((sample_number,), dtype=np.float64)
            sample_numberRead = pDaq.int32()
            self.taskCounter.ReadCounterF64(
                pDaq.DAQmx_Val_Auto, 10.0, data, sample_number, pDaq.byref(sample_numberRead), None)
            sample_numberRead = sample_numberRead.value
            if(sample_numberRead > 0):
                t = np.linspace(0, sample_numberRead / sampling_frequency, sample_numberRead)
                pos = np.mean(data)
                speed = (data[0] - data[-1]) / t[-1]
                acceleration = (data[0] - data[-1:]) / (t[-1]**2)
                self.encoderPosBuffer = np.append(self.encoderPosBuffer, pos)
                self.encoderSpeedBuffer = np.append(
                    self.encoderSpeedBuffer, speed)
                self.encoderAccelerationBuffer = np.append(
                    self.encoderAccelerationBuffer, acceleration)
                if(len(self.encoderTimeBuffer) == 0):  # Si la longueur des temps est nulle
                    # Alors on ajoute les données de temps en brut
                    self.encoderTimeBuffer = np.append(
                        self.encoderTimeBuffer, t[-1])
                else:
                    # Sinon on prend le dernier temps et on lui ajoute les
                    # valeurs de temps obtenues
                    self.encoderTimeBuffer = np.append(
                        self.encoderTimeBuffer, self.encoderTimeBuffer[-1] + t[-1])
            if(self.encoderAcquisition):
                self.counterThread = threading.Timer(
                    sample_number / sampling_frequency * 0.8, self.readTimer)
                self.counterThread.start()

    def stopEncoder(self):
        self.counterThread.join()
        self.encoderAcquisition = False
        self.TaskAI = None
        self.taskCounter.StopTask()
        self.taskCounter = None

    def getEncoderBuffer(self):
        """Lire le buffer de données de l'acquisition continue et vide le buffer en conservant la derniere valeur"""
        if(self.continuous_acquisition_flag):
            self.counterThread.join()
            if(np.shape(self.acquisitionBuffer)[1] > 1):
                buffPos = self.encoderPosBuffer[0:-1]
                buffSpe = self.encoderSpeedBuffer[0:-1]
                buffAcc = self.encoderAccelerationBuffer[0:-1]
                buffT = self.encoderTimeBuffer[0:-1]
                self.encoderPosBuffer = self.encoderPosBuffer[-1:]
                self.encoderSpeedBuffer = self.encoderSpeedBuffer[-1:]
                self.encoderAccelerationBuffer = self.encoderSpeedBuffer[-1:]
        else:
            buffPos = self.encoderPosBuffer
            buffSpe = self.encoderSpeedBuffer
            buffAcc = self.encoderAccelerationBuffer
            buffT = self.encoderTimeBuffer
        return (buffT, buffPos, buffSpe, buffAcc)

    def getDeviceName(self, voie):
        return voie.split('/')[0]

    def setSamplingClockFromAI(self, voie, frequency):
        buffer_size = 1000
        the_buffer = pDaq.create_string_buffer(buffer_size)
        dev = self.getDeviceName(voie)
        pDaq.DAQmxGetDevAIPhysicalChans(dev, the_buffer, buffer_size)
        if(the_buffer.value != ''):
            AIforSample = [str.strip(c)
                           for c in the_buffer.value.decode().split(',')][0]
        self.TaskAI = pDaq.Task()
        self.TaskAI.CreateAIVoltageChan(
            AIforSample, "", pDaq.DAQmx_Val_RSE, -10.0, 10.0, pDaq.DAQmx_Val_Volts, None)
        self.TaskAI.CfgSampClkTiming(
            "", frequency, pDaq.DAQmx_Val_Rising, pDaq.DAQmx_Val_ContSamps, 100)
#        self.TaskAI.StartTask()

    def computeFRF(self, t, sig1, sig2, fmin=-1, fmaxi=-1):
        sampling_frequency = 1 / (t[1] - t[0])
        hamm = np.hamming(len(sig1))
        sig1_fft = np.fft.fft(sig1 * hamm)
        sig1_fft = sig1_fft[0:int(len(sig1) / 2)]
        sig2_fft = np.fft.fft(sig2 * hamm)
        sig2_fft = sig2_fft[0:int(len(sig2) / 2)]
        FRF1 = (sig2_fft*np.conjugate(sig1_fft)) / (sig1_fft*np.conjugate(sig1_fft))
        FRF2 = (sig2_fft*np.conjugate(sig2_fft)) / (sig1_fft*np.conjugate(sig2_fft))

        FRF = (FRF1+FRF2) / 2

        fmax = sampling_frequency / 2
        f = np.linspace(0, fmax, len(FRF))
        if(fmin > 0):
            rangeData = f > fmin
            f = f[rangeData]
            FRF = FRF[rangeData]
        if(fmaxi > 0):

            rangeData = f < fmaxi
            f = f[rangeData]
            FRF = FRF[rangeData]
        return (f, FRF)

    def smooth(self, x, window_len=5, window="flat"):
        '''smooth the data using a window with requested size.

        This method is based on the convolution of a scaled window with the signal.
        The signal is prepared by introducing reflected copies of the signal
        (with the window size) in both ends so that transient parts are minimized
        in the begining and end part of the output signal.

        input:
        x: the input signal
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
        flat window will produce a moving average smoothing.

        output:
        the smoothed signal

        example:

        t=linspace(-2,2,0.1)
        x=sin(t)+randn(len(t))*0.1
        y=smooth(x)

        see also:

        numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
        scipy.signal.lfilter

        TODO: the window parameter could be the window itself if an array instead of a string
        NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
        '''
        def is_odd(num):
            return num % 2 != 0

        if(not(is_odd(window_len))):
            window_len = window_len - 1

        if x.ndim != 1:
            raise ValueError("smooth only accepts 1 dimension arrays.")

        if x.size < window_len:
            raise ValueError( "Input vector needs to be bigger than window size.")

        if (window_len / 2.0 - int(window_len / 2.0)) == 0:
            raise ValueError( "Window length is not a odd number.")

        if window_len < 3:
            return x
        if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
            raise ValueError("Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'")

        s = np.r_[x[window_len - 1:0:-1], x, x[-1:-window_len:-1]]

        xDim_temp = x.shape
        xDim = xDim_temp[0]

        # print(len(s))
        if window == 'flat':  # moving average
            w = np.ones(window_len, 'd')
        else:
            w = ast.literal_eval('numpy.' + window + '(window_len)')

        y = np.convolve(w / w.sum(), s, mode='valid')

        yret = np.zeros(xDim, dtype=x.dtype)
        for i in range(0, xDim):
            yret[i] = y[i + (window_len - 1) / 2]
        return yret

    def peakdetect(self, y_axis, x_axis=None, lookahead=50, delta=0):
        """
        Converted from/based on a MATLAB script at http://billauer.co.il/peakdet.html

        Algorithm for detecting local maximas and minmias in a signal.
        Discovers peaks by searching for values which are surrounded by lower
        or larger values for maximas and minimas respectively

        keyword arguments:
        y_axis -- A list containg the signal over which to find peaks
        x_axis -- A x-axis whose values correspond to the 'y_axis' list and is used
            in the return to specify the postion of the peaks. If omitted the index
            of the y_axis is used. (default: None)
        lookahead -- (optional) distance to look ahead from a peak candidate to
            determine if it is the actual peak (default: 500) 
            '(sample / period) / f' where '4 >= f >= 1.25' might be a good value
        delta -- (optional) this specifies a minimum difference between a peak and
            the following points, before a peak may be considered a peak. Useful
            to hinder the algorithm from picking up false peaks towards to end of
            the signal. To work well delta should be set to 'delta >= RMSnoise * 5'.
            (default: 0)
                Delta function causes a 20% decrease in speed, when omitted
                Correctly used it can double the speed of the algorithm

        return -- two lists [maxtab, mintab] containing the positive and negative
            peaks respectively. Each cell of the lists contains a tupple of:
            (position, peak_value) 
            to get the average peak value do 'np.mean(maxtab, 0)[1]' on the results
        """
        maxtab = []
        mintab = []
        dump = []  # Used to pop the first hit which always if false

        length = len(y_axis)
        if x_axis is None:
            x_axis = range(length)

        # perform some checks
        if length != len(x_axis):
            raise ValueError("Input vectors y_axis and x_axis must have same length")
        if lookahead < 1:
            raise ValueError("Lookahead must be above '1' in value")
        if not (np.isscalar(delta) and delta >= 0):
            raise ValueError("delta must be a positive number")

        # needs to be a numpy array
        y_axis = np.asarray(y_axis)

        # maxima and minima candidates are temporarily stored in
        # mx and mn respectively
        mn, mx = np.Inf, -np.Inf

        # Only detect peak if there is 'lookahead' amount of points after it
        for index, (x, y) in enumerate(zip(x_axis[:-lookahead], y_axis[:-lookahead])):
            if y > mx:
                mx = y
                mxpos = x
            if y < mn:
                mn = y
                mnpos = x

            ####look for max####
            if y < mx - delta and mx != np.Inf:
                # Maxima peak candidate found
                # look ahead in signal to ensure that this is a peak and not
                # jitter
                if y_axis[index:index + lookahead].max() < mx:
                    maxtab.append((mxpos, mx))
                    dump.append(True)
                    # set algorithm to only find minima now
                    mx = np.Inf
                    mn = np.Inf

            ####look for min####
            if y > mn + delta and mn != -np.Inf:
                # Minima peak candidate found
                # look ahead in signal to ensure that this is a peak and not
                # jitter
                if y_axis[index:index + lookahead].min() > mn:
                    mintab.append((mnpos, mn))
                    dump.append(False)
                    # set algorithm to only find maxima now
                    mn = -np.Inf
                    mx = -np.Inf

        # Remove the false hit on the first value of the y_axis
        try:
            if dump[0]:
                maxtab.pop(0)
                # print "pop max"
            else:
                mintab.pop(0)
                # print "pop min"
            del dump
        except IndexError:
            # no peaks were found, should the function return empty lists?
            pass

        return maxtab, mintab

    def moyennageSignals(self, sig, nmoy=3, resetMoy=False):
        if(resetMoy or self.averaging_index >= nmoy):
            self.averaging_index = 1
            self.signals_sum = np.array(sig)
        else: 
            self.signals_sum += np.array(sig)
            self.averaging_index += 1
        return(self.averaging_index == nmoy, self.signals_sum / self.averaging_index)


if __name__ == '__main__':
    print("not doing anything")
#     import time
#     mesureAnalogique=DaqMx()
#     sampling_frequency=1000 #Fréquence d'echantillonnage (Hz)
#     t=1 #Temps d'acquisition (s)
#     sample_number=t*sampling_frequency #Nombre d'echantillons par voie
#     # (t,data)=mesureAnalogique.set_analog_measurement(['/Dev1/Ai0','/Dev1/Ai1'],sampling_frequency,sample_number)
#     #Mesure 1 seconde à 1kHz
# #Mesure analogique continue
#     tdata, data = mesureAnalogique.set_analog_measurement(['/Dev1/Ai0'],sampling_frequency,typeEch='fini')
#     print(data)
