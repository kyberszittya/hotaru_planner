package hu.sze.aut.semantics.robotdescription.code.generator;

import java.io.File;
import java.util.function.Consumer;
import org.eclipse.xtext.xbase.lib.Exceptions;
import org.semanticweb.HermiT.ReasonerFactory;
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.reasoner.ConsoleProgressMonitor;
import org.semanticweb.owlapi.reasoner.InferenceType;
import org.semanticweb.owlapi.reasoner.NodeSet;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerConfiguration;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.SimpleConfiguration;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

@SuppressWarnings("all")
public class SemanticRobotDescriptionCodeGenerator {
  public static void main(final String[] args) {
    final OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
    final File robotontology = new File("ontology/robotontology.owl");
    OWLOntology localrobots = null;
    try {
      localrobots = manager.loadOntologyFromOntologyDocument(robotontology);
      System.out.println(("Loaded ontology: " + localrobots));
      final OWLReasonerFactory factory = new ReasonerFactory();
      final ConsoleProgressMonitor progressMonitor = new ConsoleProgressMonitor();
      final OWLReasonerConfiguration config = new SimpleConfiguration(progressMonitor);
      final OWLReasoner reasoner = factory.createReasoner(localrobots, config);
      reasoner.precomputeInferences(InferenceType.values());
      final DefaultPrefixManager pm = new DefaultPrefixManager(null, null, 
        "http://aut.sze.hu/RobotOntology/ontology.owl#");
      final OWLDataFactory fac = manager.getOWLDataFactory();
      final OWLClass robot = fac.getOWLClass(IRI.create(pm.getDefaultPrefix(), "Robot"));
      System.out.println(robot.getIRI());
      final NodeSet<OWLNamedIndividual> individualsNodeSet = reasoner.getInstances(robot, false);
      final Consumer<OWLDataProperty> _function = (OWLDataProperty it) -> {
        System.out.println(pm.getShortForm(it));
      };
      localrobots.dataPropertiesInSignature().forEach(_function);
    } catch (final Throwable _t) {
      if (_t instanceof OWLOntologyCreationException) {
        final OWLOntologyCreationException e = (OWLOntologyCreationException)_t;
        e.printStackTrace();
      } else {
        throw Exceptions.sneakyThrow(_t);
      }
    }
  }
}
