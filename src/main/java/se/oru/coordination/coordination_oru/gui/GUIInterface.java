/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/GUIForms/Application.java to edit this template
 */
package se.oru.coordination.coordination_oru.gui;

import addPath.ImageWindow;
import org.json.simple.parser.ParseException;

import javax.swing.*;
import java.awt.*;
import java.io.File;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;



/**
 *
 * @author bader
 */
public class GUIInterface extends javax.swing.JFrame {

    int indexOfList = 0;
    String fileName = "";
    int selectIndexTab = 0;
    int countIteration;
    long time_ms;
    
    public GUIInterface() {
        
        initialiseComponents();
        
        this.l_nameFile.setVisible(false);
        this.textBox_nameFileWrite.setVisible(false);
        this.btn_okFileName.setVisible(false);
        
        GUITools.initGUITools();
        this.panel_Robots.setVisible(false);
        
        
        this.label_repeat.setEnabled(false);
        this.textBox_repeat.setEnabled(false);
        countIteration = 100000;
        
        this.label_time.setEnabled(false);
        this.textBox_time.setEnabled(false);
        time_ms = 100000000;
        
        this.buttonBack.setEnabled(false);
        this.buttonStop.setEnabled(false);
       
            
        this.buttonNext.setEnabled(false);
        this.buttonTest.setEnabled(false);
        this.buttonSave.setEnabled(false);

    }
    
    void interfaceEnable(boolean test)
    {
        this.buttonDeleteRobot.setEnabled(test);

        this.buttonRun.setEnabled(true);
        
        StatusRobot.setEnable_iterationExperiment(true);
        StatusRobot.setEnable_endTimeOfExperiment(true);
        
        this.textBox_name.setEnabled(test);
        this.textBox_color.setEnabled(test);
        this.textBox_velocity.setEnabled(test);
        this.textBox_acceleration.setEnabled(test);
        this.textBox_size.setEnabled(test);
        this.textBox_path.setEnabled(test);
        this.addPath.setEnabled(test);
        this.textBox_iterationRobot2.setEnabled(test);
        this.buttonDeleteRobot.setEnabled(test);
        this.buttonApply.setEnabled(test);

    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code.
     * The Form Editor always regenerates the content of this method.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initialiseComponents() {

        jTabbedPane1 = new javax.swing.JTabbedPane();
        panel_Main = new javax.swing.JPanel();
        jLabel12 = new javax.swing.JLabel();
        jLabel13 = new javax.swing.JLabel();
        buttonLoad = new javax.swing.JButton();
        buttonCreate = new javax.swing.JButton();
        l_nameFile = new javax.swing.JLabel();
        textBox_nameFileWrite = new javax.swing.JTextField();
        btn_okFileName = new javax.swing.JButton();
        jPanel3 = new javax.swing.JPanel();
        jLabel7 = new javax.swing.JLabel();
        textBox_map = new javax.swing.JLabel();
        buttonChange = new javax.swing.JButton();
        l_image = new javax.swing.JLabel();
        panel_Robots = new javax.swing.JPanel();
        jLabel1 = new javax.swing.JLabel();
        jLabel2 = new javax.swing.JLabel();
        textBox_velocity = new javax.swing.JTextField();
        textBox_acceleration = new javax.swing.JTextField();
        jLabel3 = new javax.swing.JLabel();
        textBox_color = new javax.swing.JTextField();
        jLabel4 = new javax.swing.JLabel();
        jLabel5 = new javax.swing.JLabel();
        jScrollPane1 = new javax.swing.JScrollPane();
        textBox_size = new javax.swing.JTextArea();
        jLabel6 = new javax.swing.JLabel();
        jScrollPane2 = new javax.swing.JScrollPane();
        textBox_path = new javax.swing.JTextArea();
        buttonApply = new javax.swing.JButton();
        jLabel9 = new javax.swing.JLabel();
        textBox_name = new javax.swing.JTextField();
        buttonNewRobot = new javax.swing.JButton();
        buttonDeleteRobot = new javax.swing.JButton();
        jScrollPane4 = new javax.swing.JScrollPane();
        listRobot1 = new javax.swing.JList<>();
        addPath = new javax.swing.JButton();
        textBox_iterationRobot2 = new javax.swing.JTextField();
        jLabel14 = new javax.swing.JLabel();
        jPanel4 = new javax.swing.JPanel();
        jLabel11 = new javax.swing.JLabel();
        buttonRun = new javax.swing.JButton();
        jLabel10 = new javax.swing.JLabel();
        textBox_nameCSV = new javax.swing.JTextField();
        checkTime = new javax.swing.JCheckBox();
        label_time = new javax.swing.JLabel();
        textBox_time = new javax.swing.JTextField();
        textBox_repeat = new javax.swing.JTextField();
        label_repeat = new javax.swing.JLabel();
        checkRepeat = new javax.swing.JCheckBox();
        check_iteration = new javax.swing.JCheckBox();
        check_endTimeOfExp = new javax.swing.JCheckBox();
        check_namberOfCollision = new javax.swing.JCheckBox();
        check_endTimeOfRobot = new javax.swing.JCheckBox();
        check_waitingTimeForRobot = new javax.swing.JCheckBox();
        jPanel2 = new javax.swing.JPanel();
        label_log = new javax.swing.JLabel();
        jScrollPane3 = new javax.swing.JScrollPane();
        textBox_log = new javax.swing.JTextArea();
        buttonStop = new javax.swing.JButton();
        jPanel5 = new javax.swing.JPanel();
        buttonBack = new javax.swing.JButton();
        buttonSave = new javax.swing.JButton();
        buttonTest = new javax.swing.JButton();
        buttonNext = new javax.swing.JButton();
        menuBar = new javax.swing.JMenuBar();
        fileMenu = new javax.swing.JMenu();
        New_item = new javax.swing.JMenuItem();
        openMenuItem = new javax.swing.JMenuItem();
        saveMenuItem = new javax.swing.JMenuItem();
        saveAsMenuItem = new javax.swing.JMenuItem();
        exitMenuItem = new javax.swing.JMenuItem();
        helpMenu = new javax.swing.JMenu();
        contentsMenuItem = new javax.swing.JMenuItem();
        aboutMenuItem = new javax.swing.JMenuItem();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setTitle("Coordination_ORU Interface");
        setResizable(false);
        addWindowListener(new java.awt.event.WindowAdapter() {
            public void windowClosing(java.awt.event.WindowEvent evt) {
                formWindowClosing(evt);
            }
        });

        jTabbedPane1.setEnabled(false);

        panel_Main.setBackground(new java.awt.Color(255, 255, 255));
        panel_Main.setBorder(javax.swing.BorderFactory.createLineBorder(new java.awt.Color(0, 0, 0)));
        panel_Main.setPreferredSize(new java.awt.Dimension(473, 406));

        jLabel12.setFont(new java.awt.Font("Liberation Serif", 0, 36)); // NOI18N
        jLabel12.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        jLabel12.setText("Welcome");

        jLabel13.setFont(new java.awt.Font("Liberation Serif", 0, 24)); // NOI18N
        jLabel13.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        jLabel13.setText("Please load a project file");

        buttonLoad.setFont(new java.awt.Font("Liberation Sans", 0, 20)); // NOI18N
        buttonLoad.setText("Load");
        buttonLoad.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonLoad(evt);
            }
        });

        buttonCreate.setFont(new java.awt.Font("Liberation Sans", 0, 20)); // NOI18N
        buttonCreate.setText("Create");
        buttonCreate.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonCreate(evt);
            }
        });

        l_nameFile.setFont(new java.awt.Font("Liberation Serif", 0, 24)); // NOI18N
        l_nameFile.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        l_nameFile.setText("Enter file name");

        textBox_nameFileWrite.setFont(new java.awt.Font("Liberation Sans", 0, 18)); // NOI18N
        textBox_nameFileWrite.setHorizontalAlignment(javax.swing.JTextField.CENTER);
        textBox_nameFileWrite.setText("file.json");
        textBox_nameFileWrite.setBorder(javax.swing.BorderFactory.createLineBorder(new java.awt.Color(0, 0, 0)));

        btn_okFileName.setFont(new java.awt.Font("Liberation Sans", 0, 18)); // NOI18N
        btn_okFileName.setText("OK");
        btn_okFileName.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btn_okFileNameActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout panel_MainLayout = new javax.swing.GroupLayout(panel_Main);
        panel_Main.setLayout(panel_MainLayout);
        panel_MainLayout.setHorizontalGroup(
            panel_MainLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(panel_MainLayout.createSequentialGroup()
                .addContainerGap()
                .addComponent(jLabel12, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addContainerGap())
            .addGroup(panel_MainLayout.createSequentialGroup()
                .addGap(49, 49, 49)
                .addComponent(jLabel13, javax.swing.GroupLayout.DEFAULT_SIZE, 459, Short.MAX_VALUE)
                .addGap(47, 47, 47))
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, panel_MainLayout.createSequentialGroup()
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addGroup(panel_MainLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(panel_MainLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                        .addComponent(l_nameFile, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addGroup(panel_MainLayout.createSequentialGroup()
                            .addComponent(buttonCreate, javax.swing.GroupLayout.PREFERRED_SIZE, 188, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                            .addComponent(buttonLoad, javax.swing.GroupLayout.PREFERRED_SIZE, 188, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addComponent(textBox_nameFileWrite, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.PREFERRED_SIZE, 388, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(panel_MainLayout.createSequentialGroup()
                        .addGap(101, 101, 101)
                        .addComponent(btn_okFileName, javax.swing.GroupLayout.PREFERRED_SIZE, 188, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addGap(81, 81, 81))
        );
        panel_MainLayout.setVerticalGroup(
            panel_MainLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(panel_MainLayout.createSequentialGroup()
                .addGap(40, 40, 40)
                .addComponent(jLabel12, javax.swing.GroupLayout.PREFERRED_SIZE, 56, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addGap(26, 26, 26)
                .addComponent(jLabel13, javax.swing.GroupLayout.PREFERRED_SIZE, 37, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addGap(42, 42, 42)
                .addGroup(panel_MainLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(buttonLoad, javax.swing.GroupLayout.PREFERRED_SIZE, 39, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(buttonCreate, javax.swing.GroupLayout.PREFERRED_SIZE, 39, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addGap(18, 18, 18)
                .addComponent(l_nameFile, javax.swing.GroupLayout.PREFERRED_SIZE, 37, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addComponent(textBox_nameFileWrite, javax.swing.GroupLayout.PREFERRED_SIZE, 32, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addComponent(btn_okFileName)
                .addContainerGap(47, Short.MAX_VALUE))
        );

        jTabbedPane1.addTab("Project", panel_Main);

        jPanel3.setBackground(new java.awt.Color(255, 255, 255));
        jPanel3.setBorder(javax.swing.BorderFactory.createLineBorder(new java.awt.Color(0, 0, 0)));

        jLabel7.setBackground(new java.awt.Color(51, 51, 255));
        jLabel7.setFont(new java.awt.Font("Liberation Serif", 0, 24)); // NOI18N
        jLabel7.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        jLabel7.setText("Change the map");
        jLabel7.setBorder(javax.swing.BorderFactory.createEmptyBorder(1, 1, 1, 1));

        textBox_map.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        textBox_map.setText("yamlFile.yaml");
        textBox_map.setBorder(javax.swing.BorderFactory.createLineBorder(new java.awt.Color(153, 153, 153)));

        buttonChange.setText("Change");
        buttonChange.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonChange(evt);
            }
        });

        l_image.setBackground(new java.awt.Color(51, 51, 255));
        l_image.setFont(new java.awt.Font("Liberation Serif", 0, 24)); // NOI18N
        l_image.setForeground(new java.awt.Color(153, 153, 153));
        l_image.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        l_image.setText(" ");
        l_image.setBorder(javax.swing.BorderFactory.createLineBorder(new java.awt.Color(0, 0, 0)));

        javax.swing.GroupLayout jPanel3Layout = new javax.swing.GroupLayout(jPanel3);
        jPanel3.setLayout(jPanel3Layout);
        jPanel3Layout.setHorizontalGroup(
            jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel3Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(jLabel7, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel3Layout.createSequentialGroup()
                        .addComponent(textBox_map, javax.swing.GroupLayout.PREFERRED_SIZE, 346, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 12, Short.MAX_VALUE)
                        .addComponent(buttonChange, javax.swing.GroupLayout.PREFERRED_SIZE, 185, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addContainerGap())
            .addGroup(jPanel3Layout.createSequentialGroup()
                .addGap(39, 39, 39)
                .addComponent(l_image, javax.swing.GroupLayout.PREFERRED_SIZE, 471, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        jPanel3Layout.setVerticalGroup(
            jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel3Layout.createSequentialGroup()
                .addGap(26, 26, 26)
                .addComponent(jLabel7, javax.swing.GroupLayout.PREFERRED_SIZE, 29, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(textBox_map, javax.swing.GroupLayout.PREFERRED_SIZE, 29, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(buttonChange, javax.swing.GroupLayout.PREFERRED_SIZE, 29, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addGap(18, 18, 18)
                .addComponent(l_image, javax.swing.GroupLayout.PREFERRED_SIZE, 300, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );

        jTabbedPane1.addTab("Map", jPanel3);

        panel_Robots.setBackground(new java.awt.Color(255, 255, 255));
        panel_Robots.setBorder(new javax.swing.border.LineBorder(new java.awt.Color(0, 0, 0), 1, true));

        jLabel1.setFont(new java.awt.Font("Liberation Serif", 0, 24)); // NOI18N
        jLabel1.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        jLabel1.setText("Robots");

        jLabel2.setText("Maximum Velocity:");

        textBox_acceleration.setScrollOffset(1);

        jLabel3.setText("Maximum Acceleration:");

        jLabel4.setText("Color:");

        jLabel5.setText("Footprint:");

        textBox_size.setColumns(1);
        textBox_size.setLineWrap(true);
        textBox_size.setRows(2);
        textBox_size.setWrapStyleWord(true);
        jScrollPane1.setViewportView(textBox_size);

        jLabel6.setText("Path:");

        textBox_path.setColumns(1);
        textBox_path.setLineWrap(true);
        textBox_path.setRows(5);
        textBox_path.setWrapStyleWord(true);
        jScrollPane2.setViewportView(textBox_path);

        buttonApply.setText("Apply");
        buttonApply.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonApply(evt);
            }
        });

        jLabel9.setFont(new java.awt.Font("Liberation Sans", 0, 15)); // NOI18N
        jLabel9.setText("Name:");

        buttonNewRobot.setText("New Robot");
        buttonNewRobot.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonNewRobot(evt);
            }
        });

        buttonDeleteRobot.setText("Delete");
        buttonDeleteRobot.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonDeleteRobot(evt);
            }
        });

        listRobot1.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                listRobot1MouseClicked(evt);
            }
        });
        jScrollPane4.setViewportView(listRobot1);

        addPath.setText("Add");
        addPath.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                addPathActionPerformed(evt);
            }
        });

        jLabel14.setText("Iteration:");

        javax.swing.GroupLayout panel_RobotsLayout = new javax.swing.GroupLayout(panel_Robots);
        panel_Robots.setLayout(panel_RobotsLayout);
        panel_RobotsLayout.setHorizontalGroup(
            panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(panel_RobotsLayout.createSequentialGroup()
                .addContainerGap()
                .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(jLabel1, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, panel_RobotsLayout.createSequentialGroup()
                        .addComponent(jScrollPane4, javax.swing.GroupLayout.PREFERRED_SIZE, 102, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 49, Short.MAX_VALUE)
                        .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, panel_RobotsLayout.createSequentialGroup()
                                .addComponent(jLabel9)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(textBox_name, javax.swing.GroupLayout.PREFERRED_SIZE, 343, javax.swing.GroupLayout.PREFERRED_SIZE))
                            .addComponent(textBox_velocity, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.PREFERRED_SIZE, 331, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                                    .addGroup(panel_RobotsLayout.createSequentialGroup()
                                        .addComponent(jLabel4)
                                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                        .addComponent(textBox_color))
                                    .addGroup(panel_RobotsLayout.createSequentialGroup()
                                        .addComponent(jLabel3)
                                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                        .addComponent(textBox_acceleration, javax.swing.GroupLayout.PREFERRED_SIZE, 302, javax.swing.GroupLayout.PREFERRED_SIZE)))
                                .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, panel_RobotsLayout.createSequentialGroup()
                                    .addComponent(jLabel5, javax.swing.GroupLayout.PREFERRED_SIZE, 62, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                    .addComponent(jScrollPane1, javax.swing.GroupLayout.PREFERRED_SIZE, 347, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addComponent(jLabel2))
                            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                .addGroup(panel_RobotsLayout.createSequentialGroup()
                                    .addComponent(jLabel6)
                                    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                    .addComponent(jScrollPane2, javax.swing.GroupLayout.PREFERRED_SIZE, 286, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                    .addComponent(addPath, javax.swing.GroupLayout.PREFERRED_SIZE, 55, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addComponent(jLabel14, javax.swing.GroupLayout.PREFERRED_SIZE, 59, javax.swing.GroupLayout.PREFERRED_SIZE))
                            .addComponent(textBox_iterationRobot2, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.PREFERRED_SIZE, 325, javax.swing.GroupLayout.PREFERRED_SIZE)))
                    .addGroup(panel_RobotsLayout.createSequentialGroup()
                        .addComponent(buttonNewRobot)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(buttonDeleteRobot, javax.swing.GroupLayout.PREFERRED_SIZE, 82, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(buttonApply, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)))
                .addContainerGap())
        );
        panel_RobotsLayout.setVerticalGroup(
            panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(panel_RobotsLayout.createSequentialGroup()
                .addContainerGap()
                .addComponent(jLabel1, javax.swing.GroupLayout.PREFERRED_SIZE, 44, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                    .addGroup(panel_RobotsLayout.createSequentialGroup()
                        .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(textBox_name, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel9, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(textBox_velocity, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel2, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(textBox_acceleration, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel3, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(textBox_color, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel4, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                            .addComponent(jScrollPane1, javax.swing.GroupLayout.DEFAULT_SIZE, 53, Short.MAX_VALUE)
                            .addComponent(jLabel5, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                            .addComponent(jLabel6, javax.swing.GroupLayout.PREFERRED_SIZE, 41, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jScrollPane2, javax.swing.GroupLayout.PREFERRED_SIZE, 0, Short.MAX_VALUE)
                            .addComponent(addPath, javax.swing.GroupLayout.DEFAULT_SIZE, 82, Short.MAX_VALUE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(textBox_iterationRobot2, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel14, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)))
                    .addComponent(jScrollPane4, javax.swing.GroupLayout.PREFERRED_SIZE, 321, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(panel_RobotsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(buttonApply, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addComponent(buttonDeleteRobot, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addComponent(buttonNewRobot, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                .addContainerGap())
        );

        jTabbedPane1.addTab("Robots", panel_Robots);

        jPanel4.setBackground(new java.awt.Color(255, 255, 255));
        jPanel4.setBorder(javax.swing.BorderFactory.createLineBorder(new java.awt.Color(0, 0, 0)));

        jLabel11.setFont(new java.awt.Font("Liberation Serif", 0, 36)); // NOI18N
        jLabel11.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        jLabel11.setText("Experimental Setup");

        buttonRun.setText("Run");
        buttonRun.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonRun(evt);
            }
        });

        jLabel10.setFont(new java.awt.Font("Liberation Sans", 0, 15)); // NOI18N
        jLabel10.setText("CSV File Name:");

        checkTime.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                checkTimeActionPerformed(evt);
            }
        });

        label_time.setFont(new java.awt.Font("Liberation Sans", 0, 15)); // NOI18N
        label_time.setText("Experiment time(ms):");

        label_repeat.setFont(new java.awt.Font("Liberation Sans", 0, 15)); // NOI18N
        label_repeat.setText("Repeat:");

        checkRepeat.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                checkRepeatActionPerformed(evt);
            }
        });

        check_iteration.setSelected(true);
        check_iteration.setText("Iteration of experiment");
        check_iteration.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                check_iterationActionPerformed(evt);
            }
        });

        check_endTimeOfExp.setSelected(true);
        check_endTimeOfExp.setText("End time of experiment");
        check_endTimeOfExp.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                check_endTimeOfExpActionPerformed(evt);
            }
        });

        check_namberOfCollision.setText("Number of collisions");
        check_namberOfCollision.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                check_namberOfCollisionActionPerformed(evt);
            }
        });

        check_endTimeOfRobot.setText("End time of robots");
        check_endTimeOfRobot.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                check_endTimeOfRobotActionPerformed(evt);
            }
        });

        check_waitingTimeForRobot.setText("Waiting time for each robot");
        check_waitingTimeForRobot.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                check_waitingTimeForRobotActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout jPanel4Layout = new javax.swing.GroupLayout(jPanel4);
        jPanel4.setLayout(jPanel4Layout);
        jPanel4Layout.setHorizontalGroup(
            jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel4Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(buttonRun, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addComponent(jLabel11, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addGroup(jPanel4Layout.createSequentialGroup()
                        .addComponent(checkRepeat)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(label_repeat, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(textBox_repeat, javax.swing.GroupLayout.PREFERRED_SIZE, 439, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(jPanel4Layout.createSequentialGroup()
                        .addComponent(checkTime)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(label_time, javax.swing.GroupLayout.PREFERRED_SIZE, 141, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(textBox_time))
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel4Layout.createSequentialGroup()
                        .addComponent(jLabel10)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(textBox_nameCSV))
                    .addGroup(jPanel4Layout.createSequentialGroup()
                        .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(check_iteration)
                            .addComponent(check_endTimeOfRobot))
                        .addGap(18, 18, 18)
                        .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(check_waitingTimeForRobot)
                            .addComponent(check_endTimeOfExp))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addComponent(check_namberOfCollision)))
                .addContainerGap())
        );
        jPanel4Layout.setVerticalGroup(
            jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel4Layout.createSequentialGroup()
                .addGap(32, 32, 32)
                .addComponent(jLabel11, javax.swing.GroupLayout.PREFERRED_SIZE, 66, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addGap(31, 31, 31)
                .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(textBox_nameCSV, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel10, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(check_iteration)
                    .addComponent(check_endTimeOfExp)
                    .addComponent(check_namberOfCollision))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(check_endTimeOfRobot)
                    .addComponent(check_waitingTimeForRobot))
                .addGap(18, 18, 18)
                .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(label_repeat, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addComponent(textBox_repeat)
                    .addComponent(checkRepeat, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                .addGap(18, 18, 18)
                .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(checkTime, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                        .addComponent(label_time, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addComponent(textBox_time, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addGap(62, 62, 62)
                .addComponent(buttonRun, javax.swing.GroupLayout.PREFERRED_SIZE, 50, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addGap(16, 16, 16))
        );

        jTabbedPane1.addTab("Experiment", jPanel4);

        jPanel2.setBackground(new java.awt.Color(255, 255, 255));
        jPanel2.setBorder(javax.swing.BorderFactory.createLineBorder(new java.awt.Color(0, 0, 0)));

        label_log.setFont(new java.awt.Font("Liberation Serif", 0, 24)); // NOI18N
        label_log.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        label_log.setText("Log");

        textBox_log.setEditable(false);
        textBox_log.setBackground(new java.awt.Color(255, 255, 255));
        textBox_log.setColumns(1);
        textBox_log.setLineWrap(true);
        textBox_log.setRows(10);
        textBox_log.setWrapStyleWord(true);
        jScrollPane3.setViewportView(textBox_log);

        buttonStop.setText("Stop");
        buttonStop.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonStop(evt);
            }
        });

        javax.swing.GroupLayout jPanel2Layout = new javax.swing.GroupLayout(jPanel2);
        jPanel2.setLayout(jPanel2Layout);
        jPanel2Layout.setHorizontalGroup(
            jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel2Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(buttonStop, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addComponent(jScrollPane3, javax.swing.GroupLayout.DEFAULT_SIZE, 543, Short.MAX_VALUE)
                    .addComponent(label_log, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                .addContainerGap())
        );
        jPanel2Layout.setVerticalGroup(
            jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel2Layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(label_log, javax.swing.GroupLayout.PREFERRED_SIZE, 34, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(jScrollPane3, javax.swing.GroupLayout.DEFAULT_SIZE, 344, Short.MAX_VALUE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(buttonStop)
                .addContainerGap())
        );

        jTabbedPane1.addTab("Log", jPanel2);

        jPanel5.setBackground(new java.awt.Color(255, 255, 255));
        jPanel5.setBorder(javax.swing.BorderFactory.createLineBorder(new java.awt.Color(0, 0, 0)));

        buttonBack.setText("Back");
        buttonBack.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonBack(evt);
            }
        });

        buttonSave.setText("Save");
        buttonSave.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonSave(evt);
            }
        });

        buttonTest.setText("Test");
        buttonTest.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btn_TestActionPerformed(evt);
            }
        });

        buttonNext.setText("Next");
        buttonNext.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonNext(evt);
            }
        });

        javax.swing.GroupLayout jPanel5Layout = new javax.swing.GroupLayout(jPanel5);
        jPanel5.setLayout(jPanel5Layout);
        jPanel5Layout.setHorizontalGroup(
            jPanel5Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel5Layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(buttonBack, javax.swing.GroupLayout.DEFAULT_SIZE, 131, Short.MAX_VALUE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(buttonSave, javax.swing.GroupLayout.DEFAULT_SIZE, 131, Short.MAX_VALUE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(buttonTest, javax.swing.GroupLayout.DEFAULT_SIZE, 131, Short.MAX_VALUE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(buttonNext, javax.swing.GroupLayout.DEFAULT_SIZE, 132, Short.MAX_VALUE)
                .addContainerGap())
        );
        jPanel5Layout.setVerticalGroup(
            jPanel5Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel5Layout.createSequentialGroup()
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addGroup(jPanel5Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(jPanel5Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                        .addComponent(buttonTest)
                        .addComponent(buttonNext))
                    .addComponent(buttonSave)
                    .addComponent(buttonBack))
                .addContainerGap())
        );

        fileMenu.setMnemonic('f');
        fileMenu.setText("File");

        New_item.setText("New");
        New_item.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                New_itemActionPerformed(evt);
            }
        });
        fileMenu.add(New_item);

        openMenuItem.setMnemonic('o');
        openMenuItem.setText("Open");
        openMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                openMenuItemActionPerformed(evt);
            }
        });
        fileMenu.add(openMenuItem);

        saveMenuItem.setMnemonic('s');
        saveMenuItem.setText("Save");
        saveMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                saveMenuItemActionPerformed(evt);
            }
        });
        fileMenu.add(saveMenuItem);

        saveAsMenuItem.setMnemonic('a');
        saveAsMenuItem.setText("Save As ...");
        saveAsMenuItem.setDisplayedMnemonicIndex(5);
        saveAsMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                saveAsMenuItemActionPerformed(evt);
            }
        });
        fileMenu.add(saveAsMenuItem);

        exitMenuItem.setMnemonic('x');
        exitMenuItem.setText("Exit");
        exitMenuItem.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                exitMenuItemActionPerformed(evt);
            }
        });
        fileMenu.add(exitMenuItem);

        menuBar.add(fileMenu);

        helpMenu.setMnemonic('h');
        helpMenu.setText("Help");

        contentsMenuItem.setMnemonic('c');
        contentsMenuItem.setText("Contents");
        helpMenu.add(contentsMenuItem);

        aboutMenuItem.setMnemonic('a');
        aboutMenuItem.setText("About");
        helpMenu.add(aboutMenuItem);

        menuBar.add(helpMenu);

        setJMenuBar(menuBar);

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(jPanel5, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addComponent(jTabbedPane1, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(jTabbedPane1, javax.swing.GroupLayout.PREFERRED_SIZE, 459, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(jPanel5, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void exitMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_exitMenuItemActionPerformed
        System.exit(0);
    }//GEN-LAST:event_exitMenuItemActionPerformed
    
    DefaultListModel lModel = new DefaultListModel();
    private void openMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_openMenuItemActionPerformed
        // TODO add your handling code here:
        
        //################# Open File ########################
        
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setCurrentDirectory(new File(System.getProperty("user.home")));
        int result = fileChooser.showOpenDialog(this);
        if (result == JFileChooser.APPROVE_OPTION) 
        {
            File selectedFile = fileChooser.getSelectedFile();
            System.out.println("Selected file: " + selectedFile.getAbsolutePath());
            
            //listRobots.removeAll();
            fileName = selectedFile.getAbsolutePath();
            try {
                JSONFile.readDataFromJson(fileName, GUITools.pkg);
            } catch (IOException ex) {
                Logger.getLogger(GUIInterface.class.getName()).log(Level.SEVERE, null, ex);
            } catch (ParseException ex) {
                Logger.getLogger(GUIInterface.class.getName()).log(Level.SEVERE, null, ex);
            } catch (IndexOutOfBoundsException ex)
            {
                JOptionPane.showMessageDialog(null, "خطأ: لا يوجد دليل للمصفوفة", "خطأ", JOptionPane.ERROR_MESSAGE);
            }

            
            this.indexOfList=0;
            for(int i = 0; i< GUITools.pkg.robotJson.length; i++)
            {
                //listRobots.addItem("R"+(i+1));
                this.lModel.addElement("R"+(i+1));
            }
            this.listRobot1.setModel(lModel);
            this.listRobot1.setSelectedIndex(0);
            GUITools.selectListRobots(indexOfList, listRobot1, textBox_name, textBox_velocity, textBox_acceleration, textBox_color, textBox_size, textBox_path,textBox_iterationRobot2, textBox_map);
            this.interfaceEnable(true);
        }
    }//GEN-LAST:event_openMenuItemActionPerformed

    private void saveAsMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_saveAsMenuItemActionPerformed

    }//GEN-LAST:event_saveAsMenuItemActionPerformed

    private void saveMenuItemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_saveMenuItemActionPerformed
        // TODO add your handling code here:
    }//GEN-LAST:event_saveMenuItemActionPerformed
    
    private void formWindowClosing(java.awt.event.WindowEvent evt) {//GEN-FIRST:event_formWindowClosing
        // TODO add your handling code here:
        //System.out.print("BAder Jbara");
        
    }//GEN-LAST:event_formWindowClosing

    private void buttonChange(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_changeActionPerformed
        // TODO add your handling code here:
        
        //######################## Change Map ##############################
        
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setCurrentDirectory(new File(System.getProperty("user.home")));
        int result = fileChooser.showOpenDialog(this);
        if (result == JFileChooser.APPROVE_OPTION) 
        {
            File selectedFile = fileChooser.getSelectedFile();
//            System.out.println("Selected file: " + selectedFile.getAbsolutePath());
            
            String [] strs = selectedFile.getAbsolutePath().split("/");
            GUITools.pkg.yamlFile = strs[strs.length-2]+"/"+strs[strs.length-1];
            this.textBox_map.setText(strs[strs.length-2]+"/"+strs[strs.length-1]);
            
            String fileNameMap = selectedFile.getAbsolutePath().split("[.]")[0];
            ImageIcon img = new ImageIcon(fileNameMap+".png");
            Image ii = img.getImage().getScaledInstance(450, 300, Image.SCALE_SMOOTH);
            img = new ImageIcon(ii);
            this.l_image.setIcon(img);
        }
    }//GEN-LAST:event_btn_changeActionPerformed
    
    Boolean isNewRobot=true;
    private void New_itemActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_New_itemActionPerformed
        // TODO add your handling code here:
        // ######################## New #########################
        this.interfaceEnable(true);
        this.buttonStop.doClick();
        
    }//GEN-LAST:event_New_itemActionPerformed

    private void buttonDeleteRobot(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_deleteRobotActionPerformed
        // TODO add your handling code here:
        
        //######################## Delete Robot ##########################
        
        if(this.listRobot1.getModel().getSize()!=0)  // اذا وجد عناصر موجودة في قائمة الروبوتات
        {
            int index_selected = this.listRobot1.getSelectedIndex();
            GUITools.deleteRobot(index_selected);
            
            this.listRobot1.removeAll();
            this.lModel = new DefaultListModel();

            this.indexOfList = this.listRobot1.getSelectedIndex();
            //this.indexOfList=0;
            for(int i = 0; i< GUITools.pkg.robotJson.length; i++)
            {
                lModel.addElement("R"+(i+1));
            }
            this.listRobot1.setModel(lModel);
            
            //to select on JList
            this.listRobot1.setSelectedIndex(0);
            GUITools.selectListRobots(indexOfList, listRobot1, textBox_name, textBox_velocity, textBox_acceleration, textBox_color, textBox_size, textBox_path,textBox_iterationRobot2, textBox_map);
            //------------------------
        }
        
        if(this.listRobot1.getModel().getSize()==0)     //لا يوجد عناصر في قائمة الروبوتات
        {
            this.textBox_name.setText("");
            this.textBox_color.setText("");
            this.textBox_velocity.setText("");
            this.textBox_acceleration.setText("");
            this.textBox_size.setText("");
            this.textBox_path.setText("");
            
            interfaceEnable(false);
        }

    }//GEN-LAST:event_btn_deleteRobotActionPerformed

    private void buttonNewRobot(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_newRobotActionPerformed
        // TODO add your handling code here:
        
        // ######################## New Robot #########################
        
        if(isNewRobot==true) // click newRobot
        {
            buttonNewRobot.setText("Cancel");
            this.textBox_name.setText("newRobot");
            this.textBox_color.setText("");
            this.textBox_velocity.setText("4");
            this.textBox_acceleration.setText("2");
            this.textBox_size.setText("(-1.0,0.5) -> (1.0,0.5) -> (1.0,-0.5) -> (-1.0,-0.5)");
            this.textBox_path.setText("");

            this.textBox_iterationRobot2.setText("1");
            
            if(this.listRobot1.getModel().getSize()==0)
            {
                interfaceEnable(true);
                this.buttonDeleteRobot.setEnabled(false);
            }
            

        }
        else  {// click Cancel
            buttonNewRobot.setText("New Robot");
            
            if(this.listRobot1.getSelectedIndex()!=-1)
            this.listRobot1.setSelectedIndex(this.indexOfList);
            
            if(this.listRobot1.getModel().getSize()==0)
            {
                this.textBox_name.setText("");
                this.textBox_color.setText("");
                this.textBox_velocity.setText("");
                this.textBox_acceleration.setText("");
                this.textBox_size.setText("");
                this.textBox_path.setText("");

                this.textBox_iterationRobot2.setText("");

                interfaceEnable(false);
            }
            this.listRobot1.setSelectedIndex(indexOfList);
            GUITools.selectListRobots(indexOfList, listRobot1, textBox_name, textBox_velocity, textBox_acceleration, textBox_color, textBox_size, textBox_path,textBox_iterationRobot2, textBox_map);
            
        }
        isNewRobot = !isNewRobot;

    }//GEN-LAST:event_btn_newRobotActionPerformed

    private void buttonRun(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_runActionPerformed
        
        // ######################## Run ###############################

        
        this.buttonStop.setEnabled(true);
        this.buttonRun.setEnabled(false);
        this.buttonTest.setEnabled(false);
        
        if(this.checkTime.isSelected())
        {
            Timer.time_ms = Long.parseLong(this.textBox_time.getText());
        }
        else
        {
            Timer.time_ms = 100000000;
        }
        
        if(this.checkRepeat.isSelected())
        {
            countIteration=Integer.parseInt(this.textBox_repeat.getText());
        }
        else
        {
            countIteration = 100000;
        }
        
        if(!this.textBox_nameCSV.getText().isEmpty())
            StatusRobot.setFileName_CSV(this.textBox_nameCSV.getText());
        else
            StatusRobot.setFileName_CSV(this.textBox_nameFileWrite.getText().split("[.]")[0]+".csv");
        
        Timer.btn_stop = this.buttonStop;
        Timer.l_log = this.label_log;
        GUITools.run(this.textBox_log,countIteration);
        
        this.buttonNext.doClick();
        this.buttonBack.setEnabled(false);
        this.buttonNext.setEnabled(false);

    }//GEN-LAST:event_btn_runActionPerformed

    private void buttonApply(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_apply_btnActionPerformed
        // TODO add your handling code here:
        
        //########################## Apply ###############################
        
        GUITools.btn_Apply(isNewRobot, buttonNewRobot, indexOfList, listRobot1, textBox_name, textBox_velocity, textBox_acceleration, textBox_color, textBox_size, textBox_path,textBox_iterationRobot2,textBox_map);
        interfaceEnable(true);
        this.buttonTest.setEnabled(true);
        this.buttonRun.setEnabled(true);
        if(isNewRobot==false)
        {
            this.listRobot1.setSelectedIndex(this.listRobot1.getModel().getSize()-1);
            isNewRobot = true;
        }
        
    }//GEN-LAST:event_apply_btnActionPerformed

    private void listRobot1MouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_listRobot1MouseClicked
        // TODO add your handling code here:
        indexOfList=listRobot1.getSelectedIndex();
        GUITools.selectListRobots(indexOfList, listRobot1, textBox_name, textBox_velocity, textBox_acceleration, textBox_color, textBox_size, textBox_path,textBox_iterationRobot2, textBox_map);
    }//GEN-LAST:event_listRobot1MouseClicked

    private void buttonLoad(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_loadActionPerformed
        // TODO add your handling code here:
        
        //################# Open File ########################
        
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setCurrentDirectory(new File(System.getProperty("user.home")));
        int result = fileChooser.showOpenDialog(this);
        if (result == JFileChooser.APPROVE_OPTION) 
        {
            this.l_nameFile.setVisible(false);
            this.textBox_nameFileWrite.setVisible(true);
            this.textBox_nameFileWrite.setEnabled(false);
            this.btn_okFileName.setVisible(false);
        
            File selectedFile = fileChooser.getSelectedFile();
            System.out.println("Selected file: " + selectedFile.getAbsolutePath());
            
            //listRobots.removeAll();

            fileName = selectedFile.getAbsolutePath();
            try {
                JSONFile.readDataFromJson(fileName, GUITools.pkg);
                String[] fileName_aux = fileName.split("/");
            this.textBox_nameFileWrite.setText(fileName_aux[fileName_aux.length-1]);

            this.indexOfList=0; 
            lModel = new DefaultListModel();
            for(int i = 0; i< GUITools.pkg.robotJson.length; i++)
            {
                this.lModel.addElement("R"+(i+1));
            }
            this.listRobot1.setModel(lModel);

            this.listRobot1.setSelectedIndex(0);
            GUITools.selectListRobots(indexOfList, listRobot1, textBox_name, textBox_velocity, textBox_acceleration, textBox_color, textBox_size, textBox_path,textBox_iterationRobot2, textBox_map);

            this.textBox_nameCSV.setText(GUITools.pkg.csvFileName);
            this.checkRepeat.setSelected(true);
            this.textBox_repeat.setText(GUITools.pkg.experimentIteration+"");
            this.textBox_repeat.setEnabled(true);

            if(this.listRobot1.getModel().getSize()==0)     //لا يوجد عناصر في قائمة الروبوتات
            {
                interfaceEnable(false);
                this.buttonRun.setEnabled(false);

                this.buttonTest.setEnabled(false);
            }
            else
            {
                interfaceEnable(true);
                this.buttonRun.setEnabled(true);
                this.buttonNext.setEnabled(true);
                this.buttonTest.setEnabled(true);
                this.buttonSave.setEnabled(true);
            }

            String fileNameMap = GUITools.pkg.yamlFile.split("[.]")[0].split("/")[1];
            ImageIcon img = new ImageIcon(new File("").getAbsolutePath()+"/maps/"+fileNameMap+".png");
            Image ii = img.getImage().getScaledInstance(450, 300, Image.SCALE_SMOOTH);
            img = new ImageIcon(ii);
            this.l_image.setIcon(img);
            this.buttonNext(evt);  // Auto toggle to Frame 2
            } catch (IOException | ParseException ex) {
                JOptionPane.showMessageDialog(null, "Error: The file cannot be opened", "Error", JOptionPane.ERROR_MESSAGE);
                Logger.getLogger(GUIInterface.class.getName()).log(Level.SEVERE, null, ex);
            } catch (IndexOutOfBoundsException | NullPointerException ex)
            {
                JOptionPane.showMessageDialog(null, "Error: The file cannot be opened", "Error", JOptionPane.ERROR_MESSAGE);

            }

        }
    }//GEN-LAST:event_btn_loadActionPerformed

    private void buttonCreate(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_createActionPerformed
        // TODO add your handling code here:
        
        this.l_nameFile.setVisible(true);
        this.textBox_nameFileWrite.setVisible(true);
        this.textBox_nameFileWrite.setEnabled(true);
        this.textBox_nameFileWrite.setText("file.json");
        this.btn_okFileName.setVisible(true);
        this.btn_okFileName.setText("OK");
        this.btn_okFileName.setBackground(Color.white);

    }//GEN-LAST:event_btn_createActionPerformed

    private void buttonStop(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_stop1ActionPerformed
        // TODO add your handling code here:
        
        //######################## Stop #########################
        
        this.buttonStop.setEnabled(false);
        this.buttonRun.setEnabled(true);
        this.buttonTest.setEnabled(true);
        this.buttonBack.setEnabled(true);
        this.buttonNext.setEnabled(true);
        GUITools.stop();

    }//GEN-LAST:event_btn_stop1ActionPerformed

    private void buttonBack(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_BackActionPerformed
        // TODO add your handling code here:
        selectIndexTab--;
        if(selectIndexTab >= 0) {
            this.jTabbedPane1.setSelectedIndex(selectIndexTab);
            this.buttonNext.setEnabled(true);
            if (selectIndexTab == 0)
                this.buttonBack.setEnabled(false);
        }
        else {
            selectIndexTab = 0;
        }
    }//GEN-LAST:event_btn_BackActionPerformed

    private void buttonNext(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_NextActionPerformed
        // TODO add your handling code here:
        selectIndexTab++;
        if(selectIndexTab <= 4) {
            this.jTabbedPane1.setSelectedIndex(selectIndexTab);
            this.buttonBack.setEnabled(true);
            if(selectIndexTab == 4)
                this.buttonNext.setEnabled(false);
        }
        else {
            selectIndexTab=4;
            this.buttonNext.setEnabled(false);
        }
    }//GEN-LAST:event_btn_NextActionPerformed

    private void buttonSave(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_Save1ActionPerformed
        // TODO add your handling code here:
        
        // ######################### Save ############################
        GUITools.pkg.csvFileName = this.textBox_nameCSV.getText();
        if(this.checkRepeat.isSelected())
        {
            GUITools.pkg.experimentIteration = Long.parseLong(this.textBox_repeat.getText());
        }
        else
        {
            GUITools.pkg.experimentIteration = (long) 1;
        }
        GUITools.btn_save(fileName);
    }//GEN-LAST:event_btn_Save1ActionPerformed

    private void checkTimeActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_checkTimeActionPerformed
        // TODO add your handling code here:
        if(checkTime.isSelected())
        {
            this.label_time.setEnabled(true);
            this.textBox_time.setEnabled(true);
            time_ms = 10000;
            this.textBox_time.setText(time_ms+"");
        }
        else
        {
            this.label_time.setEnabled(false);
            this.textBox_time.setEnabled(false);
            time_ms = 100000000;
            this.textBox_time.setText("");
        }
    }//GEN-LAST:event_checkTimeActionPerformed

    private void checkRepeatActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_checkRepeatActionPerformed
        // TODO add your handling code here:
        if(checkRepeat.isSelected())
        {
            this.label_repeat.setEnabled(true);
            this.textBox_repeat.setEnabled(true);
            countIteration= (int) GUITools.pkg.experimentIteration;
            this.textBox_repeat.setText(countIteration+"");
        }
        else
        {
            this.label_repeat.setEnabled(false);
            this.textBox_repeat.setEnabled(false);
            countIteration=100000;
            this.textBox_repeat.setText("");
        }
    }//GEN-LAST:event_checkRepeatActionPerformed

    private void btn_TestActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_TestActionPerformed
        // TODO add your handling code here:
        this.buttonStop.setEnabled(true);
        this.buttonRun.setEnabled(false);
        this.buttonTest.setEnabled(false);
        this.buttonBack.setEnabled(false);
        
        Timer.time_ms = 100000000;
        Timer.btn_stop = this.buttonStop;
        Timer.l_log = this.label_log;
        GUITools.run(this.textBox_log,100000);
        
        selectIndexTab=4;
        this.jTabbedPane1.setSelectedIndex(selectIndexTab);
    }//GEN-LAST:event_btn_TestActionPerformed

    private void btn_okFileNameActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btn_okFileNameActionPerformed
        // TODO add your handling code here:
        if(!this.textBox_nameFileWrite.getText().isEmpty())
        {
            this.interfaceEnable(true);

            GUITools.pkg.yamlFile="";
            while(this.listRobot1.getModel().getSize()!=0)
            {
                this.buttonDeleteRobot.doClick();
            }
            this.textBox_map.setText("yamlFile.yaml");
            this.fileName=this.textBox_nameFileWrite.getText();
            GUITools.pkg.robotJson = new RobotJson[0];
            this.l_image.setIcon(null);
            
            this.textBox_nameFileWrite.setEnabled(false);
            this.btn_okFileName.setText("Done");
//            this.btn_okFileName.setBackground(Color.green);
            this.btn_okFileName.setBackground(Color.white);
            this.l_nameFile.setVisible(false);
            
            if(this.listRobot1.getModel().getSize()==0)     //لا يوجد عناصر في قائمة الروبوتات
            {
                interfaceEnable(false);
                this.buttonRun.setEnabled(false);
            }
            this.buttonNext.setEnabled(true);
            this.buttonTest.setEnabled(false);
            this.buttonSave.setEnabled(true);
        }
        else
        {
            this.btn_okFileName.setBackground(Color.red);
        }
    }//GEN-LAST:event_btn_okFileNameActionPerformed

    private void check_namberOfCollisionActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_check_namberOfCollisionActionPerformed
        // TODO add your handling code here:
        StatusRobot.setEnable_numberOfCollisions(this.check_namberOfCollision.isSelected());
    }//GEN-LAST:event_check_namberOfCollisionActionPerformed

    private void check_iterationActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_check_iterationActionPerformed
        // TODO add your handling code here:
        StatusRobot.setEnable_iterationExperiment(this.check_iteration.isSelected());
    }//GEN-LAST:event_check_iterationActionPerformed

    private void check_endTimeOfExpActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_check_endTimeOfExpActionPerformed
        // TODO add your handling code here:
        StatusRobot.setEnable_endTimeOfExperiment(this.check_endTimeOfExp.isSelected());
    }//GEN-LAST:event_check_endTimeOfExpActionPerformed

    private void check_endTimeOfRobotActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_check_endTimeOfRobotActionPerformed
        // TODO add your handling code here:
        StatusRobot.setEnable_endTimeOfRobot(this.check_endTimeOfRobot.isSelected());
    }//GEN-LAST:event_check_endTimeOfRobotActionPerformed

    private void check_waitingTimeForRobotActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_check_waitingTimeForRobotActionPerformed
        // TODO add your handling code here:
        StatusRobot.setEnable_waitingTimeOfRobot(this.check_waitingTimeForRobot.isSelected());
    }//GEN-LAST:event_check_waitingTimeForRobotActionPerformed

    private void addPathActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_addPathActionPerformed
        // TODO add your handling code here:
        String fileNameMap = GUITools.pkg.yamlFile.split("[.]")[0]+".png";
        fileNameMap = new File("").getAbsolutePath()+"/"+fileNameMap;
        ImageWindow.addPath(fileNameMap);
        ImageWindow.textArea = textBox_path;
        
    }//GEN-LAST:event_addPathActionPerformed
    
    
    

    
    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        /* Set the Nimbus look and feel */
        //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /* If Nimbus (introduced in Java SE 6) is not available, stay with the default look and feel.
         * For details see http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html 
         */
        try {
            for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager.getInstalledLookAndFeels()) {
                if ("Nimbus".equals(info.getName())) {
                    javax.swing.UIManager.setLookAndFeel(info.getClassName());
                    break;
                }
            }
        } catch (ClassNotFoundException ex) {
            java.util.logging.Logger.getLogger(GUIInterface.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (InstantiationException ex) {
            java.util.logging.Logger.getLogger(GUIInterface.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (IllegalAccessException ex) {
            java.util.logging.Logger.getLogger(GUIInterface.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (javax.swing.UnsupportedLookAndFeelException ex) {
            java.util.logging.Logger.getLogger(GUIInterface.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        }
        //</editor-fold>
        //</editor-fold>

        /* Create and display the form */
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                new GUIInterface().setVisible(true);
            }
        });
    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JMenuItem New_item;
    private javax.swing.JMenuItem aboutMenuItem;
    private javax.swing.JButton addPath;
    public javax.swing.JButton buttonApply;
    private javax.swing.JButton buttonBack;
    private javax.swing.JButton buttonNext;
    private javax.swing.JButton buttonSave;
    private javax.swing.JButton buttonTest;
    public javax.swing.JButton buttonChange;
    private javax.swing.JButton buttonCreate;
    public javax.swing.JButton buttonDeleteRobot;
    private javax.swing.JButton buttonLoad;
    public javax.swing.JButton buttonNewRobot;
    private javax.swing.JButton btn_okFileName;
    public javax.swing.JButton buttonRun;
    private javax.swing.JButton buttonStop;
    private javax.swing.JCheckBox checkRepeat;
    private javax.swing.JCheckBox checkTime;
    private javax.swing.JCheckBox check_endTimeOfExp;
    private javax.swing.JCheckBox check_endTimeOfRobot;
    private javax.swing.JCheckBox check_iteration;
    private javax.swing.JCheckBox check_namberOfCollision;
    private javax.swing.JCheckBox check_waitingTimeForRobot;
    private javax.swing.JMenuItem contentsMenuItem;
    private javax.swing.JMenuItem exitMenuItem;
    private javax.swing.JMenu fileMenu;
    private javax.swing.JMenu helpMenu;
    private javax.swing.JLabel jLabel1;
    private javax.swing.JLabel jLabel10;
    private javax.swing.JLabel jLabel11;
    private javax.swing.JLabel jLabel12;
    private javax.swing.JLabel jLabel13;
    private javax.swing.JLabel jLabel14;
    private javax.swing.JLabel jLabel2;
    private javax.swing.JLabel jLabel3;
    private javax.swing.JLabel jLabel4;
    private javax.swing.JLabel jLabel5;
    private javax.swing.JLabel jLabel6;
    private javax.swing.JLabel jLabel7;
    private javax.swing.JLabel jLabel9;
    private javax.swing.JPanel jPanel2;
    private javax.swing.JPanel jPanel3;
    private javax.swing.JPanel jPanel4;
    private javax.swing.JPanel jPanel5;
    private javax.swing.JScrollPane jScrollPane1;
    private javax.swing.JScrollPane jScrollPane2;
    private javax.swing.JScrollPane jScrollPane3;
    private javax.swing.JScrollPane jScrollPane4;
    private javax.swing.JTabbedPane jTabbedPane1;
    private javax.swing.JLabel l_image;
    private javax.swing.JLabel l_nameFile;
    private javax.swing.JLabel label_log;
    private javax.swing.JLabel label_repeat;
    private javax.swing.JLabel label_time;
    public javax.swing.JList<String> listRobot1;
    private javax.swing.JMenuBar menuBar;
    private javax.swing.JMenuItem openMenuItem;
    private javax.swing.JPanel panel_Main;
    private javax.swing.JPanel panel_Robots;
    private javax.swing.JMenuItem saveAsMenuItem;
    private javax.swing.JMenuItem saveMenuItem;
    public javax.swing.JTextField textBox_acceleration;
    public javax.swing.JTextField textBox_color;
    public javax.swing.JTextField textBox_iterationRobot2;
    public javax.swing.JTextArea textBox_log;
    private javax.swing.JLabel textBox_map;
    public javax.swing.JTextField textBox_name;
    public javax.swing.JTextField textBox_nameCSV;
    private javax.swing.JTextField textBox_nameFileWrite;
    public javax.swing.JTextArea textBox_path;
    public javax.swing.JTextField textBox_repeat;
    public javax.swing.JTextArea textBox_size;
    public javax.swing.JTextField textBox_time;
    public javax.swing.JTextField textBox_velocity;
    // End of variables declaration//GEN-END:variables

}
